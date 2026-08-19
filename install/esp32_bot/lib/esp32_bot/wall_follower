#!/usr/bin/env python3
"""Stop-scan-decide-move controller for the servo-mounted ToF sensor.

At each decision cycle the robot:
  1. Stops and waits for the servo to complete a FULL double sweep
     (5 -> 105 degrees, then 105 -> 5 degrees) so every angle bin has
     been sampled at least REQUIRED_PASSES times since the stop began.
  2. Chooses: forward if a whole front sector is clear; else turn right if
     a right sector is clear; else turn left.
  3. Moves one very short encoder-odometry segment (servo frozen, ToF not
     sampled — see the ESP32 firmware's servo_set_robot_moving()).
  4. Repeats.

The mapper (tof_mapper.py) subscribes to the same /scan topic and only
sees genuinely fresh data because scanning is fully paused during moves.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class State:
    WAITING_FOR_DATA = 'WAITING_FOR_DATA'
    SCAN = 'SCAN'
    MOVE_FORWARD = 'MOVE_FORWARD'
    TURN = 'TURN'


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')

        # Motion and decision parameters.
        # The ToF is deliberately paused during movement, so each movement
        # segment must be short enough that the previous scan remains safe.
        self.declare_parameter('step_distance', 0.05)       # metres
        self.declare_parameter('forward_speed', 0.025)      # metres/second
        self.declare_parameter('turn_speed', 0.20)           # rad/second
        self.declare_parameter('turn_angle', 1.5708)         # 90 degrees
        # Include the robot body and braking margin, not just the sensor's
        # minimum range. These values should be increased if the chassis is
        # wider/longer than the current prototype.
        self.declare_parameter('front_clear_distance', 0.22)
        self.declare_parameter('right_clear_distance', 0.18)
        self.declare_parameter('left_clear_distance', 0.18)
        self.declare_parameter('front_sector_half_angle_deg', 15.0)
        self.declare_parameter('right_sector_min_deg', 5.0)
        self.declare_parameter('right_sector_max_deg', 35.0)
        self.declare_parameter('left_sector_min_deg', 75.0)
        self.declare_parameter('left_sector_max_deg', 105.0)
        # A complete 5..105..5 double sweep can take several seconds: the
        # servo advances only as fast as the ToF can produce measurements.
        # Do not issue a movement command halfway through the reverse pass.
        self.declare_parameter('scan_wait_timeout', 20.0)
        self.declare_parameter('stop_settle_time', 0.25)
        self.declare_parameter('command_hz', 10.0)

        # How many times EVERY angle bin must be refreshed since the scan
        # phase began before the robot is allowed to move. 2 = one full
        # round trip (5->105, then 105->5), matching the intended
        # "double sweep before moving" behavior. Bins that never see an
        # obstacle (open space) will never satisfy this and instead fall
        # back on scan_wait_timeout — that's expected, not a bug.
        self.declare_parameter('required_passes', 2)

        # Scan-buffer positions correspond to the physical servo commands.
        self.declare_parameter('servo_min_deg', 5.0)
        self.declare_parameter('servo_max_deg', 105.0)
        self.declare_parameter('right_servo_deg', 5.0)
        self.declare_parameter('center_servo_deg', 55.0)
        self.declare_parameter('left_servo_deg', 105.0)
        self.declare_parameter('range_change_epsilon', 0.005)

        self.step_distance = float(self.get_parameter('step_distance').value)
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.turn_angle = float(self.get_parameter('turn_angle').value)
        self.front_clear_distance = float(
            self.get_parameter('front_clear_distance').value)
        self.right_clear_distance = float(
            self.get_parameter('right_clear_distance').value)
        self.left_clear_distance = float(
            self.get_parameter('left_clear_distance').value)
        self.front_sector_half_angle_deg = float(
            self.get_parameter('front_sector_half_angle_deg').value)
        self.right_sector_min_deg = float(
            self.get_parameter('right_sector_min_deg').value)
        self.right_sector_max_deg = float(
            self.get_parameter('right_sector_max_deg').value)
        self.left_sector_min_deg = float(
            self.get_parameter('left_sector_min_deg').value)
        self.left_sector_max_deg = float(
            self.get_parameter('left_sector_max_deg').value)
        self.scan_wait_timeout = float(self.get_parameter('scan_wait_timeout').value)
        self.stop_settle_time = float(self.get_parameter('stop_settle_time').value)
        command_hz = float(self.get_parameter('command_hz').value)
        self.required_passes = int(self.get_parameter('required_passes').value)

        self.servo_min_deg = float(self.get_parameter('servo_min_deg').value)
        self.servo_max_deg = float(self.get_parameter('servo_max_deg').value)
        self.right_servo_deg = float(self.get_parameter('right_servo_deg').value)
        self.center_servo_deg = float(self.get_parameter('center_servo_deg').value)
        self.left_servo_deg = float(self.get_parameter('left_servo_deg').value)
        self.range_change_epsilon = float(
            self.get_parameter('range_change_epsilon').value)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 20)
        self.create_timer(1.0 / command_hz, self.control_tick)

        self.state = State.WAITING_FOR_DATA
        self.latest_scan = None
        self.previous_scan = None
        # Per-index count of fresh readings seen since the current scan
        # phase began. Replaces the old "3 targeted bins" readiness check —
        # readiness now requires FULL buffer coverage, twice.
        self.scan_update_counts = []
        self.scan_deadline = None
        self.settle_deadline = None

        self.have_odom = False
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.move_start_x = 0.0
        self.move_start_y = 0.0
        self.turn_start_yaw = 0.0
        self.turn_direction = 1.0
        self.motion_deadline = None

        self.get_logger().info(
            'Stop-scan-decide-move wall follower ready; '
            'press the motor-enable button to begin.')

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.have_odom = True

    def scan_callback(self, msg: LaserScan):
        current = list(msg.ranges)

        # (Re)size the per-index counter array if the scan geometry changes
        # or this is the first message we've ever seen.
        if len(self.scan_update_counts) != len(current):
            self.scan_update_counts = [0] * len(current)

        if self.state == State.SCAN:
            if self.previous_scan is not None and len(self.previous_scan) == len(current):
                for index, value in enumerate(current):
                    # A bin only counts as a fresh sample when its new value
                    # is a valid range. The firmware writes NaN into a bin
                    # until the servo has actually revisited that angle
                    # since the scan phase began, so NaN->NaN is correctly
                    # never counted here.
                    if math.isfinite(value) and self.range_changed(
                            self.previous_scan[index], value):
                        self.scan_update_counts[index] += 1
            else:
                # First message of this scan phase (or geometry just
                # changed) — any finite value is a fresh reading.
                for index, value in enumerate(current):
                    if math.isfinite(value):
                        self.scan_update_counts[index] += 1

        self.previous_scan = current
        self.latest_scan = msg

    def range_changed(self, old_value, new_value):
        old_valid = math.isfinite(old_value)
        new_valid = math.isfinite(new_value)
        if old_valid != new_valid:
            return True
        return old_valid and abs(new_value - old_value) >= self.range_change_epsilon

    def servo_angle_index(self, servo_angle_deg, count):
        if count <= 1 or self.servo_max_deg <= self.servo_min_deg:
            return 0
        fraction = ((servo_angle_deg - self.servo_min_deg) /
                    (self.servo_max_deg - self.servo_min_deg))
        fraction = max(0.0, min(1.0, fraction))
        return int(round(fraction * (count - 1)))

    def sector_range(self, servo_min_deg, servo_max_deg):
        """Return the closest valid return in a physical servo sector.

        Taking the minimum is intentional: a single close return must prevent
        a collision, while a distant return cannot hide an obstacle elsewhere
        in the sector.
        """
        if self.latest_scan is None or not self.latest_scan.ranges:
            return None

        count = len(self.latest_scan.ranges)
        first = self.servo_angle_index(servo_min_deg, count)
        last = self.servo_angle_index(servo_max_deg, count)
        if first > last:
            first, last = last, first
        values = []
        for candidate in range(first, last + 1):
            value = self.latest_scan.ranges[candidate]
            if math.isfinite(value) and value >= self.latest_scan.range_min:
                values.append(value)
        return min(values) if values else None

    def begin_scan(self):
        self.publish_stop()
        self.state = State.SCAN
        self.scan_deadline = self.now_seconds() + self.scan_wait_timeout

        # Reset the per-index freshness counters — this scan phase must
        # accumulate its own full double-sweep before it's trusted.
        count = len(self.latest_scan.ranges) if self.latest_scan is not None else 0
        self.scan_update_counts = [0] * count

    def scan_is_ready(self):
        if not self.scan_update_counts:
            return False
        # Every single bin must have been freshly sampled at least
        # required_passes times — i.e. a genuine full sweep in each
        # direction, not just the 3 decision angles.
        return min(self.scan_update_counts) >= self.required_passes

    def begin_forward_segment(self):
        self.move_start_x = self.robot_x
        self.move_start_y = self.robot_y
        self.motion_deadline = self.now_seconds() + max(
            2.0, 2.0 * self.step_distance / max(self.forward_speed, 0.001))
        self.state = State.MOVE_FORWARD

    def begin_turn(self, direction):
        self.turn_start_yaw = self.robot_yaw
        self.turn_direction = direction
        self.motion_deadline = self.now_seconds() + max(
            5.0, 2.0 * self.turn_angle / max(self.turn_speed, 0.001))
        self.state = State.TURN

    def decide(self):
        if self.latest_scan is None:
            self.begin_scan()
            return

        front = self.sector_range(
            self.center_servo_deg - self.front_sector_half_angle_deg,
            self.center_servo_deg + self.front_sector_half_angle_deg)
        right = self.sector_range(
            self.right_sector_min_deg, self.right_sector_max_deg)
        left = self.sector_range(
            self.left_sector_min_deg, self.left_sector_max_deg)

        self.get_logger().info(
            f'decision scan: right={right} center={front} left={left}')

        # Forward has priority whenever the centre is usable and clear.
        if front is not None and front >= self.front_clear_distance:
            self.get_logger().info(
                f'forward clear ({front:.2f} m): moving {self.step_distance:.2f} m')
            self.begin_forward_segment()
            return

        # Forward blocked: choose right if that direction is available.
        if right is not None and right >= self.right_clear_distance:
            self.get_logger().info('forward blocked, right clear: turning right')
            self.begin_turn(-1.0)
            return

        # Forward and right blocked: turn left. This is the final fallback.
        if left is not None and left < self.left_clear_distance:
            self.get_logger().warn(
                f'front/right/left sectors blocked (left={left:.2f} m); '
                'turning left as the only escape')
        else:
            self.get_logger().info('forward and right blocked: turning left')
        self.begin_turn(1.0)

    def control_tick(self):
        if not self.have_odom or self.latest_scan is None:
            self.publish_stop()
            return

        now = self.now_seconds()

        if self.state == State.WAITING_FOR_DATA:
            self.begin_scan()
            return

        if self.state == State.SCAN:
            self.publish_stop()
            if self.settle_deadline is not None and now < self.settle_deadline:
                return
            if self.scan_is_ready() or now >= self.scan_deadline:
                if now >= self.scan_deadline and not self.scan_is_ready():
                    self.get_logger().info(
                        'scan timed out before full double-sweep coverage — '
                        'proceeding with partial data')
                self.decide()
            return

        if self.state == State.MOVE_FORWARD:
            distance = math.hypot(
                self.robot_x - self.move_start_x,
                self.robot_y - self.move_start_y,
            )
            # Never continue indefinitely if encoder odometry stops updating.
            if self.motion_deadline is not None and now >= self.motion_deadline:
                self.get_logger().warn('forward segment timed out; stopping to rescan')
                self.publish_stop()
                self.settle_deadline = now + self.stop_settle_time
                self.state = State.SCAN
                self.scan_deadline = now + self.scan_wait_timeout
                self.scan_update_counts = [0] * len(self.scan_update_counts)
                return

            # The scan is stationary data, but this check prevents issuing a
            # new command when the last confirmed front clearance was already
            # unsafe. The next state immediately performs a fresh scan.
            front = self.sector_range(
                self.center_servo_deg - self.front_sector_half_angle_deg,
                self.center_servo_deg + self.front_sector_half_angle_deg)
            if front is None or front < self.front_clear_distance:
                self.get_logger().warn(
                    f'front clearance unsafe ({front}); stopping to rescan')
                self.publish_stop()
                self.settle_deadline = now + self.stop_settle_time
                self.state = State.SCAN
                self.scan_deadline = now + self.scan_wait_timeout
                self.scan_update_counts = [0] * len(self.scan_update_counts)
                return

            if distance >= self.step_distance:
                self.publish_stop()
                self.settle_deadline = now + self.stop_settle_time
                self.state = State.SCAN
                self.scan_deadline = now + self.scan_wait_timeout
                self.scan_update_counts = [0] * len(self.scan_update_counts)
                return

            command = Twist()
            command.linear.x = self.forward_speed
            self.cmd_pub.publish(command)
            return

        if self.state == State.TURN:
            turned = abs(wrap_angle(self.robot_yaw - self.turn_start_yaw))
            if (self.motion_deadline is not None and now >= self.motion_deadline) or \
                    turned >= self.turn_angle:
                if self.motion_deadline is not None and now >= self.motion_deadline:
                    self.get_logger().warn('turn timed out; stopping to rescan')
                self.publish_stop()
                self.settle_deadline = now + self.stop_settle_time
                self.state = State.SCAN
                self.scan_deadline = now + self.scan_wait_timeout
                self.scan_update_counts = [0] * len(self.scan_update_counts)
                return

            command = Twist()
            command.angular.z = self.turn_direction * self.turn_speed
            self.cmd_pub.publish(command)

    def stop(self):
        self.publish_stop()


def main():
    rclpy.init()
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
