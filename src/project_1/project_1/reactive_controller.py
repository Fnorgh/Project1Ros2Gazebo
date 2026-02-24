#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ReactiveController(Node):
    def __init__(self):
        super().__init__('reactive_controller')

        self.declare_parameter('scan_topic',      '/scan')
        self.declare_parameter('odom_topic',      '/odom')
        self.declare_parameter('cmd_vel_key_topic', '/cmd_vel_key')
        self.declare_parameter('stop_topic',      '/stop_request')
        self.declare_parameter('cmd_vel_topic',   '/diffdrive_controller/cmd_vel')

        self.declare_parameter('safety_m',        0.26)
        self.declare_parameter('forward_speed',   0.15)
        self.declare_parameter('key_timeout_sec', 0.25)
        self.declare_parameter('publish_hz',      20.0)

        # ── KEY FIX: use sim time so clock matches Gazebo ──────────────────────
        self.declare_parameter('use_sim_time', True)

        self.scan_topic        = self.get_parameter('scan_topic').value
        self.odom_topic        = self.get_parameter('odom_topic').value
        self.cmd_vel_key_topic = self.get_parameter('cmd_vel_key_topic').value
        self.stop_topic        = self.get_parameter('stop_topic').value
        self.cmd_vel_topic     = self.get_parameter('cmd_vel_topic').value

        self.safety_m        = float(self.get_parameter('safety_m').value)
        self.forward_speed   = float(self.get_parameter('forward_speed').value)
        self.key_timeout_sec = float(self.get_parameter('key_timeout_sec').value)
        publish_hz           = float(self.get_parameter('publish_hz').value)

        # State
        self.min_range       = float('inf')
        self.stop_active     = False
        self.last_key_cmd    = Twist()
        self.last_key_time_ns = 0

        # Subs
        self.create_subscription(LaserScan, self.scan_topic,        self.on_scan,    10)
        self.create_subscription(Odometry,  self.odom_topic,        self.on_odom,    10)
        self.create_subscription(Twist,     self.cmd_vel_key_topic, self.on_key_cmd, 10)
        self.create_subscription(Bool,      self.stop_topic,        self.on_stop,    10)

        # Pub
        self.pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)

        self.create_timer(1.0 / publish_hz, self.tick)

        self.get_logger().info(
            f"reactive_controller up (use_sim_time={self.get_parameter('use_sim_time').value}).\n"
            f"  Sub: scan={self.scan_topic}, odom={self.odom_topic}, "
            f"key={self.cmd_vel_key_topic}, stop={self.stop_topic}\n"
            f"  Pub: {self.cmd_vel_topic} (TwistStamped)"
        )

    def now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def publish_twist(self, t: Twist):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()   # ← FIX: real timestamp
        msg.twist = t
        self.pub.publish(msg)

    def on_scan(self, msg: LaserScan):
        # FIX: use math.isfinite instead of None check (ranges are floats)
        m = float('inf')
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min < r < msg.range_max:
                if r < m:
                    m = r
        self.min_range = m

    def on_odom(self, msg: Odometry):
        pass  # reserved for future use

    def on_key_cmd(self, msg: Twist):
        self.last_key_cmd     = msg
        self.last_key_time_ns = self.now_ns()

    def on_stop(self, msg: Bool):
        self.stop_active = bool(msg.data)

    def key_active(self) -> bool:
        return (self.now_ns() - self.last_key_time_ns) <= int(self.key_timeout_sec * 1e9)

    def tick(self):
        # Priority 0: STOP request (bumper)
        if self.stop_active:
            self.publish_twist(Twist())
            return

        # Priority 1: HALT (laser safety)
        if self.min_range < self.safety_m:
            self.publish_twist(Twist())
            return

        # Priority 2: Keyboard override
        if self.key_active() and (
            abs(self.last_key_cmd.linear.x)  > 1e-3
            or abs(self.last_key_cmd.angular.z) > 1e-3
        ):
            self.publish_twist(self.last_key_cmd)
            return

        # Priority 3: Drive forward autonomously
        out = Twist()
        out.linear.x = self.forward_speed
        self.publish_twist(out)


def main():
    rclpy.init()
    node = ReactiveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_twist(Twist())  # send stop on exit
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()