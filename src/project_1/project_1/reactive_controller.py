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

        self.declare_parameter('scan_topic',        '/scan')
        self.declare_parameter('odom_topic',        '/odom')
        self.declare_parameter('cmd_vel_key_topic', '/cmd_vel_key')
        self.declare_parameter('stop_topic',        '/stop_request')
        self.declare_parameter('cmd_vel_topic',     '/diffdrive_controller/cmd_vel')
        self.declare_parameter('safety_m',          0.26)
        self.declare_parameter('forward_speed',     0.15)
        self.declare_parameter('key_timeout_sec',   0.5)
        self.declare_parameter('publish_hz',        50.0)

        self.scan_topic        = self.get_parameter('scan_topic').value
        self.odom_topic        = self.get_parameter('odom_topic').value
        self.cmd_vel_key_topic = self.get_parameter('cmd_vel_key_topic').value
        self.stop_topic        = self.get_parameter('stop_topic').value
        self.cmd_vel_topic     = self.get_parameter('cmd_vel_topic').value
        self.safety_m          = float(self.get_parameter('safety_m').value)
        self.forward_speed     = float(self.get_parameter('forward_speed').value)
        self.key_timeout_sec   = float(self.get_parameter('key_timeout_sec').value)
        publish_hz             = float(self.get_parameter('publish_hz').value)

        # State
        self.min_range        = float('inf')
        self.stop_active      = False
        self.last_key_cmd     = Twist()
        self.last_key_time_ns = 0
        self.clock_ready      = False   # guard against zero-stamp publish

        # Subs
        self.create_subscription(LaserScan, self.scan_topic,        self.on_scan,    10)
        self.create_subscription(Odometry,  self.odom_topic,        self.on_odom,    10)
        self.create_subscription(Twist,     self.cmd_vel_key_topic, self.on_key_cmd, 10)
        self.create_subscription(Bool,      self.stop_topic,        self.on_stop,    10)

        self.pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        self.create_timer(1.0 / publish_hz, self.tick)

        self.get_logger().info(
            f"reactive_controller up.\n"
            f"  Sub: scan={self.scan_topic}, odom={self.odom_topic}, "
            f"key={self.cmd_vel_key_topic}, stop={self.stop_topic}\n"
            f"  Pub: {self.cmd_vel_topic} (TwistStamped)\n"
            f"  Waiting for valid clock..."
        )

    def now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def clock_valid(self) -> bool:
        """Return True once sim clock has started publishing (non-zero)."""
        if self.clock_ready:
            return True
        if self.now_ns() > 0:
            self.clock_ready = True
            self.get_logger().info("Clock ready — reactive_controller active.")
            return True
        return False

    def publish_twist(self, t: Twist):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist = t
        self.pub.publish(msg)

    def on_scan(self, msg: LaserScan):
        m = float('inf')
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min < r < msg.range_max:
                if r < m:
                    m = r
        self.min_range = m

    def on_odom(self, msg: Odometry):
        pass

    def on_key_cmd(self, msg: Twist):
        self.last_key_cmd     = msg
        self.last_key_time_ns = self.now_ns()

    def on_stop(self, msg: Bool):
        self.stop_active = bool(msg.data)

    def key_active(self) -> bool:
        return (self.now_ns() - self.last_key_time_ns) <= int(self.key_timeout_sec * 1e9)

    def tick(self):
        # Don't publish until sim clock is flowing (avoids zero-stamp warning)
        if not self.clock_valid():
            return

        # Priority 0: bumper stop — block forward motion, allow reverse/turn
        if self.stop_active:
            cmd = Twist()
            if self.key_active() and (
                self.last_key_cmd.linear.x < -1e-3        # backing away
                or abs(self.last_key_cmd.angular.z) > 1e-3  # turning
            ):
                cmd = self.last_key_cmd
            self.publish_twist(cmd)
            return

        # Priority 1: laser safety halt
        if self.min_range < self.safety_m:
            self.publish_twist(Twist())
            return

        # Priority 2: keyboard override
        # Honor any recent key command, including explicit zero (stop).
        # After key_timeout_sec of silence, autonomous driving resumes.
        if self.key_active():
            self.publish_twist(self.last_key_cmd)
            return

        # Priority 3: drive forward
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
            node.publish_twist(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()