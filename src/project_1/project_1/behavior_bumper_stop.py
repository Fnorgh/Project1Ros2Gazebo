#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

try:
    from ros_gz_interfaces.msg import Contacts
except Exception:
    Contacts = None


class BumperStop(Node):
    def __init__(self):
        super().__init__('bumper_stop')

        self.declare_parameter('contacts_topic', '/bumper_contact')
        self.declare_parameter('stop_topic', '/stop_request')
        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('stop_hold_sec', 1.0)

        self.contacts_topic = self.get_parameter('contacts_topic').value
        self.stop_topic = self.get_parameter('stop_topic').value
        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.stop_hold_sec = float(self.get_parameter('stop_hold_sec').value)

        self.pub = self.create_publisher(Bool, self.stop_topic, 10)
        self.stop_until_ns = 0

        if Contacts is None:
            self.get_logger().error("ros_gz_interfaces.msg.Contacts not available; can't read /bumper_contact.")
        else:
            self.create_subscription(Contacts, self.contacts_topic, self.on_contacts, 10)

        self.create_timer(1.0 / self.publish_hz, self.tick)
        self.get_logger().info(f"bumper_stop: {self.contacts_topic} -> {self.stop_topic} (Bool)")

    def now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def trigger_stop(self):
        self.stop_until_ns = max(self.stop_until_ns, self.now_ns() + int(self.stop_hold_sec * 1e9))

    def on_contacts(self, msg):
        contacts = getattr(msg, 'contacts', None)
        if contacts is None:
            self.trigger_stop()
            return
        if len(contacts) > 0:
            self.trigger_stop()

    def tick(self):
        stop_active = self.now_ns() < self.stop_until_ns
        self.pub.publish(Bool(data=stop_active))


def main():
    rclpy.init()
    node = BumperStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()