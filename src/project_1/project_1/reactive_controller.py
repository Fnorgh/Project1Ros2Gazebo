#!/usr/bin/env python3
"""
reactive_controller.py
======================
Reactive controller for TurtleBot 4 (Project 1).

Behavior priority (highest -> lowest):
  1. HALT     - bumper / near-collision (laser threshold)
  2. KEYBOARD - human teleop via /cmd_vel_key
  3. ESCAPE   - symmetric obstacle within 1 ft -> turn ~180 deg (fixed-action pattern)
  4. AVOID    - asymmetric obstacle within 1 ft -> reflexive turn away
  5. WANDER   - random turn (+-15 deg) after every 1 ft of forward travel
  6. DRIVE    - drive forward

Laser geometry fields (angle_min, angle_max, angle_increment) are always used
to interpret beam angles -- no fixed beam-index assumptions are made, as required
by the project guide.

Yaw is derived from /odom quaternion via yaw_from_quat(), as given in the guide.
"""

import math
import random

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


# ---------------------------------------------------------------------------
# Helper provided by project guide
# ---------------------------------------------------------------------------

def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """Convert quaternion to yaw angle in radians."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

ONE_FOOT_M = 0.3048                     # 1 ft in metres
OBSTACLE_DIST_M = ONE_FOOT_M            # trigger distance for escape/avoid
SYMMETRY_THRESH_M = 0.05               # |left_min - right_min| <= this -> symmetric
ESCAPE_ANGLE = math.pi                  # target rotation for escape (180 deg)
ESCAPE_TOL = math.radians(30)          # acceptable tolerance around 180 deg
WANDER_MAX_RAD = math.radians(15.0)   # +-15 deg random wander
FRONT_HALF_ANGLE = math.radians(90)   # beams within +-90 deg of ahead are "front"


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------

class ReactiveController(Node):
    """
    Subsumption-style reactive controller.

    Each behavior is a method returning a Twist or None.
    tick() walks the priority list and publishes the first non-None result,
    suppressing all lower-priority behaviors.
    """

    def __init__(self):
        super().__init__('reactive_controller')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('scan_topic',        '/scan')
        self.declare_parameter('odom_topic',        '/odom')
        self.declare_parameter('cmd_vel_key_topic', '/cmd_vel_key')
        self.declare_parameter('cmd_vel_topic',     '/diffdrive_controller/cmd_vel')
        self.declare_parameter('safety_m',          0.26)
        self.declare_parameter('forward_speed',     0.15)
        self.declare_parameter('turn_speed',        0.5)
        self.declare_parameter('key_timeout_sec',   0.25)
        self.declare_parameter('publish_hz',        20.0)

        scan_topic        = self.get_parameter('scan_topic').value
        odom_topic        = self.get_parameter('odom_topic').value
        cmd_vel_key_topic = self.get_parameter('cmd_vel_key_topic').value
        cmd_vel_topic     = self.get_parameter('cmd_vel_topic').value

        self.safety_m       = float(self.get_parameter('safety_m').value)
        self.forward_speed  = float(self.get_parameter('forward_speed').value)
        self.turn_speed     = float(self.get_parameter('turn_speed').value)
        self.key_timeout_ns = int(self.get_parameter('key_timeout_sec').value * 1e9)
        publish_hz          = float(self.get_parameter('publish_hz').value)

        # ── Sensor state ─────────────────────────────────────────────────────
        self.scan: LaserScan = None
        self.min_range = float('inf')

        # ── Odom / pose state ────────────────────────────────────────────────
        self.current_yaw = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0

        # ── Keyboard state ───────────────────────────────────────────────────
        self.last_key_cmd     = Twist()
        self.last_key_time_ns = 0

        # ── Behavior 3 – ESCAPE state ────────────────────────────────────────
        # Fixed-action pattern: keep turning until ~180 deg rotated, even if
        # the obstacle disappears mid-turn.
        self.escaping            = False
        self.escape_start_yaw    = 0.0
        self.escape_target_delta = 0.0   # signed angle (+ = CCW, - = CW)

        # ── Behavior 5 – WANDER state ────────────────────────────────────────
        # Accumulate distance traveled; after 1 ft, execute a random +-15 deg turn.
        self.wander_dist_accum   = 0.0
        self.wander_prev_x       = None
        self.wander_prev_y       = None
        self.wandering           = False
        self.wander_start_yaw    = 0.0
        self.wander_target_delta = 0.0   # signed angle chosen at trigger time

        # ── Publishers / subscribers ─────────────────────────────────────────
        self.pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)
        self.create_subscription(LaserScan, scan_topic,        self.on_scan,    10)
        self.create_subscription(Odometry,  odom_topic,        self.on_odom,    10)
        self.create_subscription(Twist,     cmd_vel_key_topic, self.on_key_cmd, 10)
        self.create_timer(1.0 / publish_hz, self.tick)

        self.get_logger().info(
            f'reactive_controller ready | '
            f'scan={scan_topic}  odom={odom_topic}  '
            f'key={cmd_vel_key_topic} -> {cmd_vel_topic}'
        )

    # ── Utility ──────────────────────────────────────────────────────────────

    def now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def publish_twist(self, t: Twist) -> None:
        """Wrap Twist in a zero-stamp TwistStamped and publish."""
        msg = TwistStamped()
        msg.twist = t
        self.pub.publish(msg)

    @staticmethod
    def _twist(vx: float = 0.0, wz: float = 0.0) -> Twist:
        t = Twist()
        t.linear.x  = vx
        t.angular.z = wz
        return t

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """Shortest signed angular difference a - b, result in (-pi, pi]."""
        return math.atan2(math.sin(a - b), math.cos(a - b))

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def on_scan(self, msg: LaserScan) -> None:
        self.scan = msg
        valid = [
            r for r in msg.ranges
            if math.isfinite(r) and msg.range_min < r < msg.range_max
        ]
        self.min_range = min(valid) if valid else float('inf')

    def on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self.current_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        nx = msg.pose.pose.position.x
        ny = msg.pose.pose.position.y

        # Accumulate distance for Behavior 5 (WANDER)
        if self.wander_prev_x is not None:
            self.wander_dist_accum += math.hypot(nx - self.wander_prev_x,
                                                  ny - self.wander_prev_y)
        self.wander_prev_x = nx
        self.wander_prev_y = ny
        self.pos_x = nx
        self.pos_y = ny

    def on_key_cmd(self, msg: Twist) -> None:
        self.last_key_cmd     = msg
        self.last_key_time_ns = self.now_ns()

    # ── Scan analysis ─────────────────────────────────────────────────────────

    def _front_ranges(self) -> tuple:
        """
        Split front-cone laser readings into left and right lists.

        Uses scan.angle_min, angle_max, angle_increment -- no fixed indices.
        Left  = non-negative beam angles (robot's left)
        Right = negative beam angles    (robot's right)
        Returns (left_ranges, right_ranges).
        """
        if self.scan is None:
            return [], []

        msg   = self.scan
        left  = []
        right = []
        angle = msg.angle_min

        for r in msg.ranges:
            if abs(angle) <= FRONT_HALF_ANGLE:
                if math.isfinite(r) and msg.range_min < r < msg.range_max:
                    if angle >= 0.0:
                        left.append(r)
                    else:
                        right.append(r)
            angle += msg.angle_increment

        return left, right

    def _obstacle_in_front(self, left, right) -> bool:
        """True if any front-cone reading is within OBSTACLE_DIST_M."""
        all_front = left + right
        return bool(all_front) and min(all_front) <= OBSTACLE_DIST_M

    def _is_symmetric(self, left, right) -> bool:
        """
        True when both sides have valid readings and the closest reading on
        each side differs by no more than SYMMETRY_THRESH_M.
        """
        if not left or not right:
            return False
        return abs(min(left) - min(right)) <= SYMMETRY_THRESH_M

    # ── Behaviors ─────────────────────────────────────────────────────────────

    def _behavior_halt(self) -> Twist:
        """
        Priority 1 – HALT
        Publish zero velocity when the global laser minimum breaches the
        safety threshold (approximates bumper collision per the guide).
        """
        if self.min_range < self.safety_m:
            return self._twist()   # all-zero = stop
        return None

    def _behavior_keyboard(self) -> Twist:
        """
        Priority 2 – KEYBOARD
        Forward the most recent keyboard command if it is fresh and non-zero.
        """
        if (self.now_ns() - self.last_key_time_ns) > self.key_timeout_ns:
            return None
        cmd = self.last_key_cmd
        if abs(cmd.linear.x) > 1e-3 or abs(cmd.angular.z) > 1e-3:
            return cmd
        return None

    def _behavior_escape(self) -> Twist:
        """
        Priority 3 – ESCAPE  (fixed-action pattern)

        Trigger: obstacles are SYMMETRIC within 1 ft in front of the robot.
        Action : rotate approximately 180 deg (+-30 deg tolerance).

        This is a fixed-action pattern -- the turn continues until complete
        even if the obstacle disappears, matching the project specification.
        Turn direction is chosen randomly at trigger time.
        """
        left, right = self._front_ranges()

        # Continue an already-started escape regardless of current scan
        if self.escaping:
            rotated = abs(self._angle_diff(self.current_yaw, self.escape_start_yaw))
            if rotated >= ESCAPE_ANGLE - ESCAPE_TOL:
                self.escaping = False
                return None
            wz = math.copysign(self.turn_speed, self.escape_target_delta)
            return self._twist(wz=wz)

        # Trigger escape when obstacle is symmetric
        if self._obstacle_in_front(left, right) and self._is_symmetric(left, right):
            self.escaping            = True
            self.escape_start_yaw    = self.current_yaw
            direction                = random.choice([1.0, -1.0])
            self.escape_target_delta = math.copysign(ESCAPE_ANGLE, direction)
            self.get_logger().info(
                f'ESCAPE triggered | left_min={min(left):.2f} right_min={min(right):.2f}'
            )
            wz = math.copysign(self.turn_speed, self.escape_target_delta)
            return self._twist(wz=wz)

        return None

    def _behavior_avoid(self) -> Twist:
        """
        Priority 4 – AVOID  (reflex)

        Trigger: obstacles are ASYMMETRIC within 1 ft in front of the robot.
        Action : reflexively turn AWAY from the closer side.

        This is a pure reflex -- it stops immediately when the obstacle
        condition clears, matching the project specification.

        Laser geometry fields are used directly; no fixed indices assumed.
        """
        left, right = self._front_ranges()

        if not self._obstacle_in_front(left, right):
            return None

        # Symmetric obstacles are handled by escape, not avoid
        if self._is_symmetric(left, right):
            return None

        min_left  = min(left)  if left  else float('inf')
        min_right = min(right) if right else float('inf')

        if min_left < min_right:
            # Closer on left -> turn right (negative angular z)
            wz = -self.turn_speed
        else:
            # Closer on right -> turn left (positive angular z)
            wz = self.turn_speed

        return self._twist(wz=wz)

    def _behavior_wander(self) -> Twist:
        """
        Priority 5 – WANDER

        After every 1 ft of cumulative forward travel, pick a uniformly
        random angle in +-15 deg and rotate by that amount before resuming
        forward driving. Distance accumulation resets after each turn.
        """
        # Finish an in-progress wander turn
        if self.wandering:
            rotated = abs(self._angle_diff(self.current_yaw, self.wander_start_yaw))
            if rotated >= abs(self.wander_target_delta):
                self.wandering         = False
                self.wander_dist_accum = 0.0
                return None
            wz = math.copysign(self.turn_speed, self.wander_target_delta)
            return self._twist(wz=wz)

        # Trigger a new wander turn after 1 ft traveled
        if self.wander_dist_accum >= ONE_FOOT_M:
            angle_rad = random.uniform(-WANDER_MAX_RAD, WANDER_MAX_RAD)

            # Edge case: if sampled angle is effectively zero, skip the turn
            if abs(angle_rad) < 1e-4:
                self.wander_dist_accum = 0.0
                return None

            self.wandering           = True
            self.wander_start_yaw    = self.current_yaw
            self.wander_target_delta = angle_rad
            self.get_logger().info(
                f'WANDER triggered | '
                f'dist={self.wander_dist_accum:.3f} m  '
                f'turn={math.degrees(angle_rad):.1f} deg'
            )
            wz = math.copysign(self.turn_speed, self.wander_target_delta)
            return self._twist(wz=wz)

        return None

    def _behavior_drive(self) -> Twist:
        """
        Priority 6 – DRIVE  (always active, lowest-priority fallback)
        Simply drives forward at the configured speed.
        """
        return self._twist(vx=self.forward_speed)

    # ── Control loop ──────────────────────────────────────────────────────────

    def tick(self) -> None:
        """
        Evaluate all behaviors in strict priority order.
        The first non-None result is published; all lower-priority behaviors
        are suppressed for this tick (subsumption architecture).
        """
        cmd = (
            self._behavior_halt()       # 1 - highest
            or self._behavior_keyboard()     # 2
            or self._behavior_escape()       # 3
            or self._behavior_avoid()        # 4
            or self._behavior_wander()       # 5
            or self._behavior_drive()        # 6 - always returns a Twist
        )
        self.publish_twist(cmd)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = ReactiveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_twist(Twist())   # zero-velocity on shutdown
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()