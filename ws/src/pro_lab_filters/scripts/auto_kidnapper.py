#!/usr/bin/env python3
# Reproducible "kidnapped robot" publisher: emits /kidnap_pose at scheduled
# sim-time instants so headless batches produce identical kidnap events
# across seeds. Replaces the RViz Kidnap tool (manual click) for paper
# experiments.
#
# Schedule format (parameter `kidnap_schedule`, flat array of doubles, stride
# 4): [t1, x1, y1, yaw1, t2, x2, y2, yaw2, ...]
#   ti  = sim-seconds since this node started spinning (after `start_delay_s`)
#   xi  = target world x [m]
#   yi  = target world y [m]
#   yawi= target world yaw [rad]
#
# Empty schedule = no-op (so the same node can be launched unconditionally
# for every scenario; only `kidnapped` actually sets a non-empty schedule).
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped


class AutoKidnapper(Node):
    def __init__(self):
        super().__init__('auto_kidnapper')
        # Empty default — but Python's [] is inferred as BYTE_ARRAY, which
        # clashes with the YAML's DOUBLE_ARRAY value. Force the type via the
        # ParameterDescriptor by passing a single-element default that fixes
        # the inferred type, then treat 0-element values as "no schedule".
        self.declare_parameter('kidnap_schedule', [0.0])
        self.declare_parameter('start_delay_s', 0.0)
        self.declare_parameter('frame_id', 'map')

        # robot_teleporter subscribes /kidnap_pose with reliable + transient_local
        # — match that so the warp survives subscriber-restart timing.
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, '/kidnap_pose', qos)

        sched_flat = list(self.get_parameter('kidnap_schedule').value or [])
        # 1-element list = the placeholder default → treat as empty.
        if len(sched_flat) == 1:
            sched_flat = []
        if len(sched_flat) % 4 != 0:
            self.get_logger().warn(
                f'kidnap_schedule length {len(sched_flat)} is not a multiple of 4 '
                '(stride: t, x, y, yaw); dropping the tail')
            sched_flat = sched_flat[: len(sched_flat) - (len(sched_flat) % 4)]

        # List of (t_s, x, y, yaw); sort by time so we can pop the front.
        self._schedule = sorted(
            [(sched_flat[i], sched_flat[i + 1], sched_flat[i + 2], sched_flat[i + 3])
             for i in range(0, len(sched_flat), 4)],
            key=lambda e: e[0],
        )
        if not self._schedule:
            self.get_logger().info('auto_kidnapper: empty schedule — no-op')
        else:
            self.get_logger().info(
                f'auto_kidnapper: {len(self._schedule)} kidnap event(s) scheduled')

        self._t0 = None
        self._next_idx = 0
        self._start_delay_s = float(self.get_parameter('start_delay_s').value)
        self.create_timer(0.1, self._tick)  # 10 Hz polling against sim clock

    def _tick(self):
        if self._next_idx >= len(self._schedule):
            return
        # Use sim time. First tick captures t0; thereafter we compare elapsed.
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = now_s
            return
        elapsed = now_s - self._t0 - self._start_delay_s
        while (self._next_idx < len(self._schedule)
               and self._schedule[self._next_idx][0] <= elapsed):
            t, x, y, yaw = self._schedule[self._next_idx]
            self._publish(x, y, yaw)
            self.get_logger().info(
                f'kidnap #{self._next_idx + 1} fired @ t={elapsed:.1f}s '
                f'(scheduled {t:.1f}s): -> ({x:.2f}, {y:.2f}, {yaw:.2f} rad)')
            self._next_idx += 1

    def _publish(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        # stamp=0 so robot_teleporter's QoS / TF lookup uses "latest available".
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(AutoKidnapper())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
