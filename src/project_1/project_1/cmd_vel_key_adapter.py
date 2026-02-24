#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class CmdVelKeyAdapter(Node):
    def __init__(self):
        super().__init__('cmd_vel_key_adapter')
        self.declare_parameter('in_topic', '/cmd_vel_key')
        self.declare_parameter('out_topic', '/diffdrive_controller/cmd_vel')

        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value

        self.pub = self.create_publisher(TwistStamped, out_topic, 10)
        self.create_subscription(Twist, in_topic, self.on_twist, 10)
        self.get_logger().info(f"{in_topic} (Twist) -> {out_topic} (TwistStamped)")

    def on_twist(self, msg: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.twist = msg
        self.pub.publish(out)

def main():
    rclpy.init()
    node = CmdVelKeyAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()