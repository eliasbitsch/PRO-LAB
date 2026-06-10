#!/usr/bin/env python3
# Publish a one-shot /initialpose for AMCL so its lifecycle can leave the
# "Please set the initial pose..." state and start particle filtering.
#
# Reads the same scenario init params (init_x/y/yaw/cov + spread) that the
# other filters consume, so AMCL starts from the *same* (possibly wrong)
# pose as KF/EKF/PF — that is the whole point of the wrong-init study.
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped


class AmclInitPosePublisher(Node):
    def __init__(self):
        super().__init__('amcl_init_pose')
        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_yaw', 0.0)
        self.declare_parameter('init_cov', 0.25)
        self.declare_parameter('frame_id', 'map')
        # Delay so AMCL has time to come up via the lifecycle manager before
        # we shove the initial pose at it. nav2_bringup activates AMCL ~3-5s
        # after launch start, the filters_group fires at +10s, so 6s is safe.
        self.declare_parameter('delay_s', 6.0)
        # AMCL silently drops the initial pose when its TF cache hasn't quite
        # caught up to the message timestamp ("Failed to transform initial
        # pose in time"). So we keep republishing on `period_s` until AMCL
        # stops emitting the "Please set the initial pose..." warning — i.e.
        # we just spam a few attempts. `attempts` caps the resend count.
        self.declare_parameter('period_s', 2.0)
        self.declare_parameter('attempts', 6)

        # AMCL's /initialpose subscription is TRANSIENT_LOCAL — using default
        # VOLATILE here triggers "incompatible QoS, no messages will be sent"
        # and AMCL silently never sees the initial pose.
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', qos)
        self._sent_count = 0
        self._attempts = int(self.get_parameter('attempts').value)
        delay = float(self.get_parameter('delay_s').value)
        self.create_timer(delay, self._first_publish)

    def _first_publish(self):
        if self._sent_count > 0:
            return
        self._publish_once()
        period = float(self.get_parameter('period_s').value)
        self.create_timer(period, self._publish_once)

    def _publish_once(self):
        if self._sent_count >= self._attempts:
            return
        x   = self.get_parameter('init_x').value
        y   = self.get_parameter('init_y').value
        yaw = self.get_parameter('init_yaw').value
        cov = self.get_parameter('init_cov').value

        msg = PoseWithCovarianceStamped()
        # stamp=0 → TF uses "latest available" instead of trying to extrapolate
        # to the exact wall-clock instant. Without this AMCL hits
        # "Lookup would require extrapolation into the future" on the first
        # initialPoseReceived and never publishes the map→odom TF, so
        # /amcl_pose stays silent for the whole run.
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # 6x6 row-major: diagonal xx, yy, yaw-yaw at indices 0, 7, 35.
        msg.pose.covariance[0]  = cov
        msg.pose.covariance[7]  = cov
        msg.pose.covariance[35] = cov

        self.pub.publish(msg)
        self._sent_count += 1
        self.get_logger().info(
            f'sent /initialpose [{self._sent_count}/{self._attempts}]: '
            f'x={x:.2f} y={y:.2f} yaw={yaw:.2f} cov={cov:.2f}')


def main():
    rclpy.init()
    rclpy.spin(AmclInitPosePublisher())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
