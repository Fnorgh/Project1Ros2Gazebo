"""
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
from rclpy.qos import qos_profile_sensor_data

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

ONE_FOOT_M = 0.3048
OBSTACLE_DIST_M = ONE_FOOT_M
SYMMETRY_THRESH_M = 0.08

ESCAPE_ANGLE = math.pi
ESCAPE_TOL = math.radians(30.0)

WANDER_MAX_RAD = math.radians(15.0)

FRONT_CENTER_HALF_ANGLE = math.radians(20.0)
FRONT_SIDE_HALF_ANGLE = math.radians(60.0)


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

        # Parameters
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_key_topic', '/cmd_vel_key')
        self.declare_parameter('cmd_vel_topic', '/diffdrive_controller/cmd_vel')
        self.declare_parameter('safety_m', 0.26)
        self.declare_parameter('forward_speed', 0.15)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('key_timeout_sec', 0.25)
        self.declare_parameter('publish_hz', 20.0)

        scan_topic = self.get_parameter('scan_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        cmd_vel_key_topic = self.get_parameter('cmd_vel_key_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.safety_m = float(self.get_parameter('safety_m').value)
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.key_timeout_ns = int(float(self.get_parameter('key_timeout_sec').value) * 1e9)
        publish_hz = float(self.get_parameter('publish_hz').value)

        # Sensor state
        self.scan: LaserScan | None = None
        self.min_range = float('inf')

        # Odom / pose state
        self.current_yaw = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0

        # Keyboard state
        self.last_key_cmd = Twist()
        self.last_key_time_ns = 0

        # Behavior 3 - ESCAPE state
        self.escaping = False
        self.escape_start_yaw = 0.0
        self.escape_target_delta = 0.0

        # Behavior 5 - WANDER state
        self.wander_dist_accum = 0.0
        self.wander_prev_x = None
        self.wander_prev_y = None
        self.wandering = False
        self.wander_start_yaw = 0.0
        self.wander_target_delta = 0.0

        # Publishers / subscribers
        self.pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.create_subscription(Twist, cmd_vel_key_topic, self.on_key_cmd, 10)
        self.create_timer(1.0 / publish_hz, self.tick)

        self.get_logger().info(
            f'reactive_controller ready | '
            f'scan={scan_topic}  odom={odom_topic}  '
            f'key={cmd_vel_key_topic} -> {cmd_vel_topic}'
        )

    # -----------------------------------------------------------------------
    # Utility
    # -----------------------------------------------------------------------

    def now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def publish_twist(self, t: Twist) -> None:
        """
        Publish a zero-stamp TwistStamped.

        The diff_drive_controller in ROS 2 control accepts TwistStamped on
        ~/cmd_vel and uses linear.x and angular.z as the body velocity command.
        :contentReference[oaicite:0]{index=0}
        """
        msg = TwistStamped()
        msg.twist = t
        self.pub.publish(msg)

    @staticmethod
    def _twist(vx: float = 0.0, wz: float = 0.0) -> Twist:
        t = Twist()
        t.linear.x = vx
        t.angular.z = wz
        return t

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """Shortest signed angular difference a - b, in (-pi, pi]."""
        return math.atan2(math.sin(a - b), math.cos(a - b))

    @staticmethod
    def _sector_min(ranges) -> float:
        return min(ranges) if ranges else float('inf')

    def _front_obstacle_present(self, left, center, right) -> bool:
        return min(
            self._sector_min(left),
            self._sector_min(center),
            self._sector_min(right),
        ) <= OBSTACLE_DIST_M

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def on_scan(self, msg: LaserScan) -> None:
        self.scan = msg

        valid = [
            r for r in msg.ranges
            if math.isfinite(r) and msg.range_min < r < msg.range_max
        ]
        self.min_range = min(valid) if valid else float('inf')

        if not hasattr(self, '_scan_seen'):
            self._scan_seen = True
            self.get_logger().info(
                f'SCAN OK | beams={len(msg.ranges)} '
                f'range_min={msg.range_min:.3f} range_max={msg.range_max:.3f} '
                f'min_range={self.min_range:.3f}'
            )

    def on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self.current_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        nx = msg.pose.pose.position.x
        ny = msg.pose.pose.position.y

        if self.wander_prev_x is not None:
            self.wander_dist_accum += math.hypot(
                nx - self.wander_prev_x,
                ny - self.wander_prev_y,
            )

        self.wander_prev_x = nx
        self.wander_prev_y = ny
        self.pos_x = nx
        self.pos_y = ny

    def on_key_cmd(self, msg: Twist) -> None:
        self.last_key_cmd = msg
        self.last_key_time_ns = self.now_ns()

    # -----------------------------------------------------------------------
    # Scan analysis
    # -----------------------------------------------------------------------

    def _front_sectors(self):
        """
        Return (left, center, right) valid ranges from the forward field of view.

        left   :  +20 deg to +60 deg
        center :  -20 deg to +20 deg
        right  :  -60 deg to -20 deg

        LaserScan angles are interpreted using angle_min and angle_increment,
        with 0 radians forward along the x-axis. :contentReference[oaicite:1]{index=1}
        """
        if self.scan is None:
            return [], [], []

        msg = self.scan
        left = []
        center = []
        right = []

        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min < r < msg.range_max:
                if -FRONT_CENTER_HALF_ANGLE <= angle <= FRONT_CENTER_HALF_ANGLE:
                    center.append(r)
                elif FRONT_CENTER_HALF_ANGLE < angle <= FRONT_SIDE_HALF_ANGLE:
                    left.append(r)
                elif -FRONT_SIDE_HALF_ANGLE <= angle < -FRONT_CENTER_HALF_ANGLE:
                    right.append(r)
            angle += msg.angle_increment

        return left, center, right

    # -----------------------------------------------------------------------
    # Behaviors
    # -----------------------------------------------------------------------

    def _behavior_halt(self) -> Twist | None:
        """
        Priority 1 – HALT
        Stop if the nearest valid scan range breaches the safety threshold.
        """
        if self.min_range < self.safety_m:
            return self._twist()
        return None

    def _behavior_keyboard(self) -> Twist | None:
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

    def _behavior_escape(self) -> Twist | None:
        """
        Priority 3 – ESCAPE (fixed-action pattern)

        Trigger: obstacle within 1 ft in front-center and left/right are
        roughly symmetric.
        Action : committed ~180 deg turn (±30 deg), continuing until completed.
        """
        if self.escaping:
            rotated = abs(self._angle_diff(self.current_yaw, self.escape_start_yaw))
            if rotated >= abs(self.escape_target_delta):
                self.escaping = False
                self.get_logger().info('ESCAPE complete')
                return None

            wz = math.copysign(self.turn_speed, self.escape_target_delta)
            return self._twist(vx=0.0, wz=wz)

        left, center, right = self._front_sectors()

        center_min = self._sector_min(center)
        left_min = self._sector_min(left)
        right_min = self._sector_min(right)

        if center_min > OBSTACLE_DIST_M:
            return None

        if not left or not right:
            return None

        if abs(left_min - right_min) <= SYMMETRY_THRESH_M:
            mag = random.uniform(ESCAPE_ANGLE - ESCAPE_TOL, ESCAPE_ANGLE + ESCAPE_TOL)
            sign = random.choice([-1.0, 1.0])

            self.escaping = True
            self.escape_start_yaw = self.current_yaw
            self.escape_target_delta = sign * mag

            self.get_logger().info(
                f'ESCAPE triggered | '
                f'C={center_min:.3f} L={left_min:.3f} R={right_min:.3f} '
                f'turn={math.degrees(self.escape_target_delta):.1f} deg'
            )

            wz = math.copysign(self.turn_speed, self.escape_target_delta)
            return self._twist(vx=0.0, wz=wz)

        return None

    def _behavior_avoid(self) -> Twist | None:
        """
        Priority 4 – AVOID (reflex)

        Trigger: obstacle within 1 ft ahead, but not symmetric enough for ESCAPE.
        Action : turn away from the closer side while obstacle persists.
        """
        left, center, right = self._front_sectors()

        left_min = self._sector_min(left)
        center_min = self._sector_min(center)
        right_min = self._sector_min(right)

        if min(left_min, center_min, right_min) > OBSTACLE_DIST_M:
            return None

        # Let ESCAPE handle the symmetric head-on case
        if center_min <= OBSTACLE_DIST_M and left and right:
            if abs(left_min - right_min) <= SYMMETRY_THRESH_M:
                return None

        avoid_deadband = 0.02

        if left_min + avoid_deadband < right_min:
            wz = -self.turn_speed
        elif right_min + avoid_deadband < left_min:
            wz = self.turn_speed
        else:
            if left and not right:
                wz = -self.turn_speed
            elif right and not left:
                wz = self.turn_speed
            else:
                wz = self.turn_speed

        self.get_logger().info(
            f'AVOID | L={left_min:.3f} C={center_min:.3f} R={right_min:.3f} wz={wz:.2f}'
        )

        return self._twist(vx=0.0, wz=wz)

    def _behavior_wander(self) -> Twist | None:
        """
        Priority 5 – WANDER

        After every 1 ft of cumulative forward travel, pick a uniformly random
        angle in ±15 deg and rotate by that amount before resuming forward
        driving. Distance accumulation resets after each turn.
        """
        if self.wandering:
            rotated = abs(self._angle_diff(self.current_yaw, self.wander_start_yaw))
            if rotated >= abs(self.wander_target_delta):
                self.wandering = False
                self.wander_dist_accum = 0.0
                return None

            wz = math.copysign(self.turn_speed, self.wander_target_delta)
            return self._twist(vx=0.0, wz=wz)

        if self.wander_dist_accum >= ONE_FOOT_M:
            angle_rad = random.uniform(-WANDER_MAX_RAD, WANDER_MAX_RAD)

            if abs(angle_rad) < 1e-4:
                self.wander_dist_accum = 0.0
                return None

            self.wandering = True
            self.wander_start_yaw = self.current_yaw
            self.wander_target_delta = angle_rad

            self.get_logger().info(
                f'WANDER triggered | '
                f'dist={self.wander_dist_accum:.3f} m  '
                f'turn={math.degrees(angle_rad):.1f} deg'
            )

            wz = math.copysign(self.turn_speed, self.wander_target_delta)
            return self._twist(vx=0.0, wz=wz)

        return None

    def _behavior_drive(self) -> Twist:
        """
        Priority 6 – DRIVE
        Lowest-priority fallback: drive forward.
        """
        return self._twist(vx=self.forward_speed, wz=0.0)

    # -----------------------------------------------------------------------
    # Control loop
    # -----------------------------------------------------------------------

    def tick(self) -> None:
        """
        Evaluate behaviors in strict priority order.
        """
        cmd = (
            self._behavior_halt()
            or self._behavior_keyboard()
            or self._behavior_escape()
            or self._behavior_avoid()
            or self._behavior_wander()
            or self._behavior_drive()
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
            node.publish_twist(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
