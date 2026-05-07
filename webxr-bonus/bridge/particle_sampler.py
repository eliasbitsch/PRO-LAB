"""Standalone ROS2 node — samples N particles from /pf/pose covariance.

Does NOT touch filter code. Reads the existing PoseWithCovarianceStamped on
/pf/pose, draws N samples from the Gaussian, publishes as PoseArray on
/webxr/particles. The WebXR app subscribes to that.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import (PoseWithCovarianceStamped, PoseArray, Pose,
                               Quaternion)


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class ParticleSampler(Node):
    def __init__(self):
        super().__init__('particle_sampler')
        self.declare_parameter('num_particles', 80)
        self.declare_parameter('min_xy_std', 0.05)
        self.declare_parameter('min_yaw_std', 0.03)
        self.declare_parameter('input_topic', '/pf/pose')
        self.declare_parameter('output_topic', '/webxr/particles')

        self.N = int(self.get_parameter('num_particles').value)
        self.min_xy = float(self.get_parameter('min_xy_std').value)
        self.min_yaw = float(self.get_parameter('min_yaw_std').value)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(PoseArray, out_topic, qos)
        self.create_subscription(PoseWithCovarianceStamped, in_topic,
                                 self.cb, qos)
        self.rng = np.random.default_rng()
        self.get_logger().info(
            f'sampling {self.N} particles from {in_topic} -> {out_topic}')

    def cb(self, msg: PoseWithCovarianceStamped):
        cov = np.array(msg.pose.covariance).reshape(6, 6)
        # PoseWithCovariance covariance is [x, y, z, rx, ry, rz]
        sx = max(math.sqrt(max(cov[0, 0], 0.0)), self.min_xy)
        sy = max(math.sqrt(max(cov[1, 1], 0.0)), self.min_xy)
        syaw = max(math.sqrt(max(cov[5, 5], 0.0)), self.min_yaw)

        # Center yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw0 = math.atan2(siny_cosp, cosy_cosp)
        x0 = msg.pose.pose.position.x
        y0 = msg.pose.pose.position.y

        out = PoseArray()
        out.header = msg.header
        for _ in range(self.N):
            p = Pose()
            p.position.x = float(x0 + self.rng.normal(0.0, sx))
            p.position.y = float(y0 + self.rng.normal(0.0, sy))
            p.position.z = 0.0
            p.orientation = yaw_to_quat(yaw0 + self.rng.normal(0.0, syaw))
            out.poses.append(p)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ParticleSampler()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
