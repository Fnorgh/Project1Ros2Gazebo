#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

try:
    from ros_gz_interfaces.msg import Contacts
except Exception:
    Contacts = None


class BumperStop(Node):
    def __init__(self):
        super().__init__('bumper_stop')

        # Inputs
        self.declare_parameter('contacts_topic', '/bumper_contact')

        # Output (THIS is the important part for your sim)
        self.declare_parameter('cmd_vel_topic', '/diffdrive_controller/cmd_vel')

        # Timing
        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('stop_hold_sec', 1.0)

        self.contacts_topic = self.get_parameter('contacts_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.stop_hold_sec = float(self.get_parameter('stop_hold_sec').value)

        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.stop_until_ns = 0

        if Contacts is None:
            self.get_logger().error(
                "ros_gz_interfaces.msg.Contacts not available; can't read /bumper_contact."
            )
        else:
            self.create_subscription(Contacts, self.contacts_topic, self.on_contacts, 10)

        self.create_timer(1.0 / self.publish_hz, self.tick)

        self.get_logger().info(
            f"bumper_stop: {self.contacts_topic} -> STOP on {self.cmd_vel_topic} (Twist)"
        )

    def now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def trigger_stop(self):
        self.stop_until_ns = max(
            self.stop_until_ns,
            self.now_ns() + int(self.stop_hold_sec * 1e9),
        )

    def on_contacts(self, msg):
        contacts = getattr(msg, 'contacts', None)

        # If schema differs, fail-safe: stop on any message
        if contacts is None:
            self.trigger_stop()
            return

        if len(contacts) > 0:
            self.trigger_stop()

    def tick(self):
        if self.now_ns() < self.stop_until_ns:
            self.pub.publish(Twist())  # all zeros


def main():
    rclpy.init()
    node = BumperStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()