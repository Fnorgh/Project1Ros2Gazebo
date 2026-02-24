#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class ReactiveController(Node):
    def __init__(self):
        super().__init__('reactive_controller')

        # Topics required by spec
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_key_topic', '/cmd_vel_key')

        # IMPORTANT: real sim drive topic + stamped message type
        self.declare_parameter('cmd_vel_topic', '/diffdrive_controller/cmd_vel')

        # Behavior params
        self.declare_parameter('safety_m', 0.26)          # collision threshold
        self.declare_parameter('forward_speed', 0.15)
        self.declare_parameter('key_timeout_sec', 0.25)   # how long keyboard cmd stays active
        self.declare_parameter('publish_hz', 20.0)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_vel_key_topic = self.get_parameter('cmd_vel_key_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.safety_m = float(self.get_parameter('safety_m').value)
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.key_timeout_sec = float(self.get_parameter('key_timeout_sec').value)
        publish_hz = float(self.get_parameter('publish_hz').value)

        # State
        self.min_range = float('inf')
        self.last_key_cmd = Twist()
        self.last_key_time_ns = 0

        # Subs
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)  # required by spec
        self.create_subscription(Twist, self.cmd_vel_key_topic, self.on_key_cmd, 10)

        # Pub (TwistStamped!)
        self.pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)

        self.create_timer(1.0 / publish_hz, self.tick)

        self.get_logger().info(
            f"reactive_controller up. Sub: {self.scan_topic}, {self.odom_topic}, {self.cmd_vel_key_topic}. "
            f"Pub: {self.cmd_vel_topic} (TwistStamped, zero-stamp)"
        )

    def now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def publish_twist(self, t: Twist):
        # KEY FIX: leave header.stamp ZERO so diffdrive_controller stamps it internally
        msg = TwistStamped()
        msg.twist = t
        self.pub.publish(msg)

    def on_scan(self, msg: LaserScan):
        # robust min range ignoring invalid values
        m = float('inf')
        for r in msg.ranges:
            if r is None:
                continue
            if r > msg.range_min and r < msg.range_max and r < m:
                m = r
        self.min_range = m

    def on_odom(self, msg: Odometry):
        # Not used for reactive behavior here, but subscribed as required.
        pass

    def on_key_cmd(self, msg: Twist):
        self.last_key_cmd = msg
        self.last_key_time_ns = self.now_ns()

    def key_active(self) -> bool:
        return (self.now_ns() - self.last_key_time_ns) <= int(self.key_timeout_sec * 1e9)

    def tick(self):
        out = Twist()

        # Priority 1: HALT
        if self.min_range < self.safety_m:
            self.publish_twist(out)  # zeros
            return

        # Priority 2: Keyboard command
        if self.key_active() and (
            abs(self.last_key_cmd.linear.x) > 1e-3
            or abs(self.last_key_cmd.angular.z) > 1e-3
        ):
            self.publish_twist(self.last_key_cmd)
            return

        # Priority 6: Drive forward
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
            node.publish_twist(Twist())  # stop
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()