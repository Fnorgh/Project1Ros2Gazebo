#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class BehaviorMux(Node):
    # Phase 2 (Part 6): drive forward behavior.

    def __init__(self):
        super().__init__('behavior_mux')

        self.declare_parameter('output_cmd_topic',    '/model/turtlebot4/cmd_vel')
        self.declare_parameter('forward_speed',        0.15)
        self.declare_parameter('publish_hz',           20.0)

        out_topic       = self.get_parameter('output_cmd_topic').value
        self.forward_speed   = float(self.get_parameter('forward_speed').value)
        publish_hz           = float(self.get_parameter('publish_hz').value)

        self.pub = self.create_publisher(TwistStamped, out_topic, 10)

        self.create_timer(1.0 / publish_hz, self.tick)

        self.get_logger().info(
            f'behavior_mux: part6 drive-forward -> {out_topic} (TwistStamped)'
        )

    # ── publish helper ─────────────────────────────────────────────────────────

    def _publish(self, tw: Twist):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist = tw
        self.pub.publish(msg)

    # ── control tick ───────────────────────────────────────────────────────────

    def tick(self):
        # (6) drive forward
        out = Twist()
        out.linear.x = self.forward_speed
        self._publish(out)


def main():
    rclpy.init()
    node = BehaviorMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish(Twist())   # send zero before exit
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
