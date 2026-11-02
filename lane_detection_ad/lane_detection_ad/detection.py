'''
lane_detection_ad node.

Same interface as the senior's lane_detection node (subscribes /camera CompressedImage,
/lane_state, /lane_mode, /intersection_state; publishes cmd_vel and
/lane_detection/debug/compressed), with the new estimator from lane_estimator.py.
Additions: /odom for the path memory, start_enabled, camera watchdog, periodic status log.
'''
import math
import os
import threading
import time

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from ament_index_python.packages import get_package_share_directory

from lane_detection_ad.lane_estimator import DEFAULTS, LaneConfig, LaneEstimator, to_bev

MODEL_FILE = 'lane_yolo26n_seg.pt'


def find_model(path):
    if path:
        return path
    for pkg in ('lane_detection_ad', 'lane_detection'):
        try:
            p = os.path.join(get_package_share_directory(pkg), 'models', MODEL_FILE)
        except Exception:
            continue
        if os.path.exists(p):
            return p
    raise RuntimeError('lane model %s not found; set the model_path parameter' % MODEL_FILE)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LaneDetection(Node):
    def __init__(self):
        super().__init__('lane_detection_ad')
        p = self.declare_parameter
        p('model_path', '')
        p('conf', 0.4)
        p('imgsz', 640)
        p('device', '')               # '' = auto, 'cpu', '0'
        p('show', True)               # cv2 window (auto-off without a display)
        p('publish_debug', True)      # /lane_detection/debug/compressed
        p('process_hz', 15.0)
        p('start_enabled', False)     # True: drive without waiting for /lane_state
        p('image_topic', '/camera')
        p('cmd_vel_topic', 'cmd_vel')
        p('odom_topic', '/odom')
        p('camera_timeout', 0.5)      # s without a processed frame -> stop
        p('log_period', 0.5)          # s
        p('profiles_file', os.path.join(get_package_share_directory('lane_detection_ad'), 'config',
                                        'section_profiles.yaml'))
        for k, v in DEFAULTS.items():
            p(k, v)

        self.show = self.get_parameter('show').value and bool(os.environ.get('DISPLAY'))
        self.conf = self.get_parameter('conf').value
        self.imgsz = self.get_parameter('imgsz').value
        self.device = self.get_parameter('device').value or None

        from ultralytics import YOLO   # imported here so the estimator stays testable without it
        model_path = find_model(self.get_parameter('model_path').value)
        self.model = YOLO(model_path, task='segment')
        self.get_logger().info('Lane model loaded: %s' % model_path)

        self.create_subscription(CompressedImage, self.get_parameter('image_topic').value,
                                 self.videoSubscriber_callback, 10)
        self.create_subscription(Bool, '/lane_mode', self.laneModeSubscriber_callback, 10)
        self.create_subscription(Bool, '/lane_state', self.stateSubscriber_callback, 10)
        self.create_subscription(Bool, '/intersection_state', self.intersectionStateSubscriber_callback, 10)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.odom_callback, 10)
        self.create_subscription(String, '/lane/section', self.section_callback, 10)
        with open(self.get_parameter('profiles_file').value) as f:
            self.profiles = yaml.safe_load(f) or {}
        self.section = 'auto'
        self.velocityPublisher = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.debugPublisher = None
        if self.get_parameter('publish_debug').value:
            self.debugPublisher = self.create_publisher(CompressedImage, '/lane_detection/debug/compressed', 10)

        self.lane_mode = True   # True = yellow (left)
        self.state = bool(self.get_parameter('start_enabled').value)
        self.is_intersection = False
        self.estimator = LaneEstimator(self.lane_config())
        self.latest_msg = None
        self.odom_pose, self.odom_t = None, None
        self.pose, self.pose_t = (0.0, 0.0, 0.0), None
        self.last_frame_t = None
        self.stopped_by_watchdog = False
        self.last_log_t = 0.0
        self.infer_ms = 0.0
        self.age_ms = float('nan')
        self.camera_timeout = self.get_parameter('camera_timeout').value
        self.log_period = self.get_parameter('log_period').value
        self.timer = self.create_timer(1.0 / self.get_parameter('process_hz').value, self.process)
        if self.state:
            self.get_logger().info('start_enabled: lane following active')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def base_params(self):
        return {k: self.get_parameter(k).value for k in DEFAULTS}

    def lane_config(self):
        cfg = LaneConfig(**self.base_params())
        for k, v in (self.profiles.get(getattr(self, 'section', 'auto')) or {}).items():
            setattr(cfg, k, v)
        return cfg

    def section_callback(self, msg):
        '''Hardcoded version: section_judge tells which section we are in -> parameter profile.'''
        if msg.data == self.section:
            return
        if msg.data not in self.profiles:
            self.get_logger().warn('no profile for section %s' % msg.data)
            return
        values = self.base_params()
        values.update(self.profiles[msg.data] or {})
        self.estimator.set_params(values)
        self.get_logger().info('section %s -> %s %s' % (self.section, msg.data, self.profiles[msg.data] or ''))
        self.section = msg.data

    # ---- senior's mode handling (unchanged behaviour) ---------------------------------

    def intersectionStateSubscriber_callback(self, msg):
        if not self.is_intersection and msg.data:
            self.is_intersection = msg.data
            self.get_logger().info('===>> Start Intersection Mode')
        if self.is_intersection and not msg.data:
            self.is_intersection = msg.data
            self.get_logger().info('===>> Finish Intersection Mode')
            if self.lane_mode:
                threading.Timer(25.0, self.intersection_left_handling).start()
            else:
                threading.Timer(12.0, self.intersection_right_handling).start()
                threading.Timer(28.0, self.intersection_right_handling2).start()

    def intersection_left_handling(self):
        self.lane_mode = False

    def intersection_right_handling(self):
        self.lane_mode = True

    def intersection_right_handling2(self):
        self.lane_mode = False

    def stateSubscriber_callback(self, msg):
        self.state = msg.data
        if self.state:
            self.estimator = LaneEstimator(self.lane_config())
            self.get_logger().info('Start Lane Detection')
        else:
            self.velocityPublisher.publish(Twist())
            self.get_logger().info('Terminate Lane Detection')

    def laneModeSubscriber_callback(self, msg):
        self.lane_mode = msg.data
        self.get_logger().info('Change lane mode to %s' % ('YELLOW' if self.lane_mode else 'WHITE'))

    # ---- inputs -------------------------------------------------------------------------

    def videoSubscriber_callback(self, msg):
        if self.state:
            self.latest_msg = msg

    def odom_callback(self, msg):
        p = msg.pose.pose
        self.odom_pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.odom_t = self.now()

    def update_pose(self, now):
        if self.odom_pose is not None and now - self.odom_t < 0.3:
            self.pose = self.odom_pose
        elif self.pose_t is not None:   # no odometry: integrate our own commands
            dt = min(max(now - self.pose_t, 0.0), 0.3)
            x, y, yaw = self.pose
            v, w = self.estimator.v_cmd, self.estimator.w_cmd
            self.pose = (x + v * dt * math.cos(yaw), y + v * dt * math.sin(yaw), yaw + w * dt)
        self.pose_t = now

    def detect(self, bev):
        '''YOLO seg inference on the BEV image -> list of (cls, mask bool HxW, conf)'''
        result = self.model.predict(bev, imgsz=self.imgsz, conf=self.conf, device=self.device,
                                    retina_masks=True, verbose=False)[0]
        if result.masks is None:
            return []
        masks = result.masks.data.cpu().numpy() > 0.5
        classes = result.boxes.cls.cpu().numpy().astype(int)
        confs = result.boxes.conf.cpu().numpy()
        return [(c, m, float(p)) for c, m, p in zip(classes, masks, confs)]

    # ---- main loop ----------------------------------------------------------------------

    def process(self):
        now = self.now()
        msg, self.latest_msg = self.latest_msg, None
        if not self.state:
            return
        if msg is None:
            if self.last_frame_t is not None and now - self.last_frame_t > self.camera_timeout \
                    and not self.stopped_by_watchdog:
                self.velocityPublisher.publish(Twist())
                self.stopped_by_watchdog = True
                self.get_logger().warn('no camera frame for %.1f s -> stop' % (now - self.last_frame_t))
            return
        self.last_frame_t, self.stopped_by_watchdog = now, False

        try:
            src = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            bev = to_bev(src)
        except Exception as e:
            self.get_logger().error('Error decoding compressed image: %s' % e)
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.age_ms = (now - stamp) * 1000.0 if stamp > 0 else float('nan')

        self.update_pose(now)
        t0 = time.monotonic()
        detections = self.detect(bev)
        self.infer_ms = (time.monotonic() - t0) * 1000.0
        v, w = self.estimator.update(detections, self.pose, now, lane_mode=self.lane_mode,
                                     intersection=self.is_intersection)
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.velocityPublisher.publish(twist)

        if self.debugPublisher is not None or self.show:
            view = self.estimator.draw(bev, detections)
            if self.debugPublisher is not None:
                debug = CompressedImage()
                debug.header = msg.header
                debug.format = 'jpeg'
                debug.data = cv2.imencode('.jpg', view)[1].tobytes()
                self.debugPublisher.publish(debug)
            if self.show:
                cv2.imshow('lane_detection_ad', view)
                cv2.waitKey(1)

        if now - self.last_log_t >= self.log_period:
            self.last_log_t = now
            self.get_logger().info('[%s] ' % self.section + ' | '.join(self.estimator.debug_lines())
                                   + ' | infer=%.0fms age=%.0fms' % (self.infer_ms, self.age_ms))


def main():
    rclpy.init()
    node = LaneDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        try:
            node.velocityPublisher.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
