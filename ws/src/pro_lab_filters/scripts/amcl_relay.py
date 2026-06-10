#!/usr/bin/env python3
# Relay /amcl_pose -> /amcl/pose so AMCL fits the /<filter>/pose convention
# every other node (metrics_node, csv_logger) already speaks. Replaces the
# topic_tools/relay node, which isn't packaged for ros-jazzy.
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class AmclRelay(Node):
    def __init__(self):
        super().__init__('amcl_relay')
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, '/amcl/pose', 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pub.publish, 10)


def main():
    rclpy.init()
    rclpy.spin(AmclRelay())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
