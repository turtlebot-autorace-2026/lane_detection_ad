'''
Hardcoded version: says which section of the known map the robot is in.

Walks through config/section_table.yaml in order using odometry distance and publishes the
section type on /lane/section (std_msgs/String: straight | curve | zigzag | mission | auto).
The lane node switches its parameter profile (config/section_profiles.yaml) on that topic.

Distance alone cannot follow this map (random intersection branch, parking manoeuvre), so sections
with length_m = -1 wait for an anchor on /lane/section_anchor (std_msgs/String = section name).
The mission state machine is meant to publish those anchors; for now they can be sent by hand.
'''
import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


class SectionJudge(Node):
    def __init__(self):
        super().__init__('section_judge')
        default_table = os.path.join(get_package_share_directory('lane_detection_ad'), 'config',
                                     'section_table.yaml')
        self.declare_parameter('table_file', default_table)
        self.declare_parameter('start_section', '')     # '' = first entry
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('publish_hz', 5.0)
        with open(self.get_parameter('table_file').value) as f:
            self.table = yaml.safe_load(f)['sections']
        self.names = [s['name'] for s in self.table]
        start = self.get_parameter('start_section').value or self.names[0]
        self.index = self.lookup(start)
        if self.index is None:
            raise SystemExit('section_judge: unknown start_section %s (have %s)' % (start, self.names))
        self.dist = 0.0
        self.last_xy = None
        self.pub = self.create_publisher(String, '/lane/section', 10)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.on_odom, 10)
        self.create_subscription(String, '/lane/section_anchor', self.on_anchor, 10)
        self.create_timer(1.0 / self.get_parameter('publish_hz').value, self.tick)
        self.announce('start')

    def lookup(self, name):
        return self.names.index(name) if name in self.names else None

    def announce(self, why):
        s = self.table[self.index]
        self.get_logger().info('section %d/%d: %s (%s, %s) <- %s' % (
            self.index + 1, len(self.table), s['name'], s['type'],
            'until anchor' if s['length_m'] < 0 else '%.2f m' % s['length_m'], why))

    def on_odom(self, msg):
        p = msg.pose.pose.position
        if self.last_xy is not None:
            self.dist += math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1])
        self.last_xy = (p.x, p.y)

    def on_anchor(self, msg):
        i = self.lookup(msg.data)
        if i is None:
            self.get_logger().warn('unknown anchor %s' % msg.data)
            return
        self.index, self.dist = i, 0.0
        self.announce('anchor')

    def tick(self):
        s = self.table[self.index]
        if 0 <= s['length_m'] <= self.dist and self.index + 1 < len(self.table):
            self.index, self.dist = self.index + 1, 0.0
            self.announce('%.2f m driven' % s['length_m'])
        self.pub.publish(String(data=self.table[self.index]['type']))


def main():
    rclpy.init()
    node = SectionJudge()
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
