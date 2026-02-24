#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data  # SensorDataQoS shortcut

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class ReactiveController(Node):
    """
    Phase 2 controller (Behaviors 1 and 2):

    Priority:
      (1) HALT if collision likely (approximated using /scan min range < safety threshold)
      (2) KEYBOARD teleop passthrough from /cmd_vel_key

    Publishes TwistStamped to /diffdrive_controller/cmd_vel for TB4 sim wiring.
    """

    def __init__(self):
        super().__init__('reactive_controller')

        # Required topics (spec)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_key_topic', '/cmd_vel_key')

        # Output for THIS sim (TwistStamped)
        self.declare_parameter('cmd_out_topic', '/diffdrive_controller/cmd_vel')

        # Behavior params
        self.declare_parameter('safety_m', 0.26)          # 0.22–0.30m recommended
        self.declare_parameter('key_timeout_sec', 0.30)   # how long a key cmd stays active
        self.declare_parameter('publish_hz', 20.0)

        # Practical extras (debug/stability)
        self.declare_parameter('debug_hz', 1.0)           # how often to print min_range
        self.declare_parameter('halt_latch_sec', 0.25)    # hold STOP briefly after triggering

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.cmd_vel_key_topic = str(self.get_parameter('cmd_vel_key_topic').value)
        self.cmd_out_topic = str(self.get_parameter('cmd_out_topic').value)

        self.safety_m = float(self.get_parameter('safety_m').value)
        self.key_timeout_sec = float(self.get_parameter('key_timeout_sec').value)
        publish_hz = float(self.get_parameter('publish_hz').value)

        self.debug_hz = float(self.get_parameter('debug_hz').value)
        self.halt_latch_sec = float(self.get_parameter('halt_latch_sec').value)

        # State
        self.min_range = math.inf
        self.scan_valid_beams = 0

        self.last_key_cmd = Twist()
        self.last_key_time_ns = 0

        self.halt_until_ns = 0  # latched stop window

        # Subscribers
        # LaserScan should use sensor-data QoS (best effort / small queue)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, qos_profile_sensor_data)

        # /odom is required by spec (used later)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, QoSProfile(depth=10))

        # /cmd_vel_key is Twist from teleop (reliable is fine)
        self.create_subscription(Twist, self.cmd_vel_key_topic, self.on_key_cmd, QoSProfile(depth=10))

        # Publisher (TwistStamped)
        self.pub = self.create_publisher(TwistStamped, self.cmd_out_topic, QoSProfile(depth=10))

        # Control timer
        self.publish_period = 1.0 / publish_hz
        self.create_timer(self.publish_period, self.tick)

        # Debug throttle
        self._dbg_every_ticks = max(1, int(round(publish_hz / max(self.debug_hz, 0.1))))
        self._tick_count = 0

        self.get_logger().info(
            "reactive_controller running\n"
            f"  Sub: scan={self.scan_topic}, odom={self.odom_topic}, key={self.cmd_vel_key_topic}\n"
            f"  Pub: {self.cmd_out_topic} (TwistStamped)\n"
            f"  Params: safety_m={self.safety_m:.3f}, key_timeout={self.key_timeout_sec:.2f}s, "
            f"halt_latch={self.halt_latch_sec:.2f}s, debug_hz={self.debug_hz:.2f}"
        )

    def now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def on_scan(self, msg: LaserScan):
        """
        Robust min-range:
          - ignore NaN/Inf
          - treat 0.0 as extremely close (map to range_min)
          - clamp to [range_min, range_max]
        """
        m = math.inf
        valid = 0

        rmin = float(msg.range_min)
        rmax = float(msg.range_max)

        for r in msg.ranges:
            if r is None:
                continue
            if not math.isfinite(r):
                continue

            # Some sims return 0.0 on contact / invalid; treat as "very close"
            if r == 0.0:
                r = rmin

            # Clamp
            if r < rmin:
                r = rmin
            if r > rmax:
                continue

            valid += 1
            if r < m:
                m = r

        self.min_range = m
        self.scan_valid_beams = valid

    def on_odom(self, msg: Odometry):
        # Not used in behaviors 1/2, but required by spec and used later
        pass

    def on_key_cmd(self, msg: Twist):
        self.last_key_cmd = msg
        self.last_key_time_ns = self.now_ns()

    def key_active(self) -> bool:
        return (self.now_ns() - self.last_key_time_ns) <= int(self.key_timeout_sec * 1e9)

    def publish_stamped(self, tw: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.twist = tw
        self.pub.publish(out)

    def tick(self):
        self._tick_count += 1
        now = self.now_ns()

        # Throttled debug
        if self._tick_count % self._dbg_every_ticks == 0:
            self.get_logger().info(
                f"min_range={self.min_range:.3f} safety={self.safety_m:.3f} valid_beams={self.scan_valid_beams}"
            )

        # If we recently triggered halt, keep stopping for a short window
        if now < self.halt_until_ns:
            self.publish_stamped(Twist())
            return

        # Behavior 1: HALT if “collision” likely (scan too close)
        if self.min_range < self.safety_m:
            self.halt_until_ns = now + int(self.halt_latch_sec * 1e9)
            self.get_logger().warn(
                f"HALT: obstacle too close (min_range={self.min_range:.3f} < safety={self.safety_m:.3f})"
            )
            self.publish_stamped(Twist())
            return

        # Behavior 2: Keyboard passthrough (if recently commanded)
        if self.key_active():
            self.publish_stamped(self.last_key_cmd)
            return

        # No key command yet: publish stop (Behavior 6 later)
        self.publish_stamped(Twist())


def main():
    rclpy.init()
    node = ReactiveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_stamped(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()