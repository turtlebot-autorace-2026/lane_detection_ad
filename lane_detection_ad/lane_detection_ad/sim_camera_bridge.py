'''
Gazebo camera -> the real robot's /camera format.

The autorace_gazebo robot (burger_cam_bag) publishes an already undistorted 1280x720
sensor_msgs/Image on /camera/image_raw. The real camera_ros node publishes /camera as an
undistorted 640x480 JPEG CompressedImage. This node resizes + JPEG-encodes so the lane
nodes run unchanged in simulation.
'''
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class SimCameraBridge(Node):
    def __init__(self):
        super().__init__('sim_camera_bridge')
        self.declare_parameter('in_topic', '/camera/image_raw')
        self.declare_parameter('out_topic', '/camera')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('jpeg_quality', 90)
        self.size = (self.get_parameter('width').value, self.get_parameter('height').value)
        self.quality = int(self.get_parameter('jpeg_quality').value)
        self.pub = self.create_publisher(CompressedImage, self.get_parameter('out_topic').value, 10)
        self.create_subscription(Image, self.get_parameter('in_topic').value, self.callback,
                                 qos_profile_sensor_data)
        self.warned = False

    def callback(self, msg):
        enc = msg.encoding
        buf = np.frombuffer(msg.data, np.uint8)
        if enc in ('rgb8', 'bgr8'):
            img = buf.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
            if enc == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif enc == 'mono8':
            img = cv2.cvtColor(buf.reshape(msg.height, msg.step)[:, :msg.width], cv2.COLOR_GRAY2BGR)
        else:
            if not self.warned:
                self.get_logger().warn('unsupported encoding %s' % enc)
                self.warned = True
            return
        img = cv2.resize(img, self.size, interpolation=cv2.INTER_AREA)
        ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            return
        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data = jpg.tobytes()
        self.pub.publish(out)


def main():
    rclpy.init()
    node = SimCameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
