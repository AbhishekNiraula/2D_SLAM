#!/usr/bin/env python3
# ============================================================
#  explorer.py — autonomous single-beam-ToF exploration node
#
#  Drives forward until an obstacle is detected, stops and
#  rotates a full 360 degrees recording (heading, distance)
#  samples from the single-beam /scan, then picks the clearest
#  heading toward the least-visited nearby cell, turns to face
#  it, and resumes driving. Tracks visited odom cells so it
#  prefers new territory over retracing paths.
#
#  PATCHED for a small (45cm) arena: hard radius-from-start cap
#  and a max-leg-distance cap force frequent rotate-scans so
#  position drift can't compound into a runaway "snake" path
#  that no longer matches the physical box.
# ============================================================

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


# ─────────────────────────────────────────────
#  TUNABLE PARAMETERS
# ─────────────────────────────────────────────
FORWARD_SPEED       = 0.08   # m/s — slow, matches slow-SLAM-friendly pace
ROTATE_SPEED        = 0.35   # rad/s — slow sweep for clean per-angle samples
TURN_SPEED          = 0.45   # rad/s — turning to a chosen heading
OBSTACLE_STOP_DIST  = 0.15   # m — trigger rotate-scan when front < this (scaled for 45cm arena)
SAFE_RESUME_DIST    = 0.20   # m — chosen heading must be at least this clear
SWEEP_SAMPLE_EVERY  = math.radians(10.0)   # record a sample every ~10 deg
HEADING_TOLERANCE   = math.radians(5.0)    # "close enough" when turning
CELL_SIZE           = 0.08   # m — visited-grid resolution (scaled down for small arena)
CANDIDATE_STEP       = 0.15  # m — how far ahead a heading's target cell is checked
CMD_PUBLISH_HZ       = 10.0  # must stay well under firmware's 500ms watchdog
EXPLORE_TIMEOUT_S    = 300.0 # safety cap — force return-home after this long
RETURN_HOME_TOLERANCE = 0.10 # m — close enough to (0,0) to call it done

# ── NEW: drift-containment caps for a 45cm arena ──────────────
MAX_RADIUS_FROM_START = 0.35   # m — arena corner distance (~0.32m) + margin
MAX_LEG_DISTANCE       = 0.12   # m — force a rotate-scan well before drift can build up


class State:
    DRIVE = "DRIVE"
    ROTATE_SCAN = "ROTATE_SCAN"
    TURN_TO_HEADING = "TURN_TO_HEADING"
    RETURN_HOME = "RETURN_HOME"
    DONE = "DONE"


def wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.front_dist = OBSTACLE_STOP_DIST + 1.0  # optimistic until first reading
        self.have_odom = False
        self.have_scan = False

        self.visited = {}  # (ix, iy) -> visit count

        self.state = State.DRIVE
        self.start_time = time.time()

        # ── NEW: track distance driven since last scan-stop ──────
        self.leg_start_x = 0.0
        self.leg_start_y = 0.0

        # ROTATE_SCAN bookkeeping
        self.sweep_start_theta = 0.0
        self.sweep_rotated = 0.0
        self.sweep_last_sample_theta = 0.0
        self.sweep_samples = []  # list of (absolute_theta, distance)

        # TURN_TO_HEADING bookkeeping
        self.target_theta = 0.0

        self.timer = self.create_timer(1.0 / CMD_PUBLISH_HZ, self.tick)
        self.get_logger().info('[Explorer] started — state=DRIVE')

    # ─────────────────────────────────────────
    #  Subscriptions
    # ─────────────────────────────────────────
    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # yaw from quaternion (z, w only — firmware only ever sets those two)
        self.theta = 2.0 * math.atan2(q.z, q.w)
        self.have_odom = True
        self.mark_visited(self.x, self.y)

    def scan_cb(self, msg: LaserScan):
        valid = [r for r in msg.ranges if r > 0.0]
        if valid:
            self.front_dist = min(valid)
            self.have_scan = True

    # ─────────────────────────────────────────
    #  Visited-cell grid
    # ─────────────────────────────────────────
    def cell_of(self, x, y):
        return (int(math.floor(x / CELL_SIZE)), int(math.floor(y / CELL_SIZE)))

    def mark_visited(self, x, y):
        c = self.cell_of(x, y)
        self.visited[c] = self.visited.get(c, 0) + 1

    def visits_in_direction(self, heading):
        tx = self.x + CANDIDATE_STEP * math.cos(heading)
        ty = self.y + CANDIDATE_STEP * math.sin(heading)
        return self.visited.get(self.cell_of(tx, ty), 0)

    # ─────────────────────────────────────────
    #  Publish helper
    # ─────────────────────────────────────────
    def publish_twist(self, linear, angular):
        t = Twist()
        t.linear.x = linear
        t.angular.z = angular
        self.cmd_pub.publish(t)

    # ─────────────────────────────────────────
    #  Main state machine — runs at CMD_PUBLISH_HZ
    # ─────────────────────────────────────────
    def tick(self):
        if not (self.have_odom and self.have_scan):
            self.publish_twist(0.0, 0.0)
            return

        if self.state != State.DONE and (time.time() - self.start_time) > EXPLORE_TIMEOUT_S:
            self.get_logger().info('[Explorer] timeout reached — returning home')
            self.state = State.RETURN_HOME

        # ── NEW: hard radius cap, checked regardless of state ────
        if self.state in (State.DRIVE,) :
            dist_from_start = math.hypot(self.x, self.y)
            if dist_from_start > MAX_RADIUS_FROM_START:
                self.get_logger().info(
                    f'[Explorer] radius cap hit ({dist_from_start:.2f} m) — forcing return-home')
                self.publish_twist(0.0, 0.0)
                self.state = State.RETURN_HOME
                return

        if self.state == State.DRIVE:
            self._do_drive()
        elif self.state == State.ROTATE_SCAN:
            self._do_rotate_scan()
        elif self.state == State.TURN_TO_HEADING:
            self._do_turn_to_heading()
        elif self.state == State.RETURN_HOME:
            self._do_return_home()
        elif self.state == State.DONE:
            self.publish_twist(0.0, 0.0)

    # ── DRIVE ──────────────────────────────────
    def _do_drive(self):
        if self.front_dist < OBSTACLE_STOP_DIST:
            self.get_logger().info(
                f'[Explorer] obstacle at {self.front_dist:.2f} m — stopping, starting sweep')
            self._start_sweep()
            return

        # ── NEW: cap how far a single leg can go before checking in ──
        leg_dist = math.hypot(self.x - self.leg_start_x, self.y - self.leg_start_y)
        if leg_dist >= MAX_LEG_DISTANCE:
            self.get_logger().info(
                f'[Explorer] leg distance cap hit ({leg_dist:.2f} m) — starting sweep')
            self._start_sweep()
            return

        self.publish_twist(FORWARD_SPEED, 0.0)

    def _start_sweep(self):
        self.publish_twist(0.0, 0.0)
        self.sweep_start_theta = self.theta
        self.sweep_rotated = 0.0
        self.sweep_last_sample_theta = self.theta
        self.sweep_samples = [(self.theta, self.front_dist)]
        self.state = State.ROTATE_SCAN

    # ── ROTATE_SCAN ────────────────────────────
    def _do_rotate_scan(self):
        # record a sample roughly every SWEEP_SAMPLE_EVERY radians of rotation
        if abs(wrap_angle(self.theta - self.sweep_last_sample_theta)) >= SWEEP_SAMPLE_EVERY:
            self.sweep_samples.append((self.theta, self.front_dist))
            self.sweep_last_sample_theta = self.theta

        self.sweep_rotated = self._cumulative_rotation()

        if self.sweep_rotated >= 2.0 * math.pi + math.radians(15):
            self.get_logger().info(
                f'[Explorer] sweep complete — {len(self.sweep_samples)} samples')
            self.publish_twist(0.0, 0.0)
            self._choose_heading_from_sweep()
            return

        self.publish_twist(0.0, ROTATE_SPEED)

    _sweep_last_theta_for_accum = None
    _sweep_accum = 0.0

    def _cumulative_rotation(self):
        if self._sweep_last_theta_for_accum is None:
            self._sweep_last_theta_for_accum = self.theta
            self._sweep_accum = 0.0
        else:
            d = wrap_angle(self.theta - self._sweep_last_theta_for_accum)
            self._sweep_accum += abs(d)
            self._sweep_last_theta_for_accum = self.theta
        return self._sweep_accum

    def _reset_sweep_accum(self):
        self._sweep_last_theta_for_accum = None
        self._sweep_accum = 0.0

    # ── choose next heading from sweep samples ─
    def _choose_heading_from_sweep(self):
        self._reset_sweep_accum()

        candidates = [s for s in self.sweep_samples if s[1] >= SAFE_RESUME_DIST]
        if not candidates:
            candidates = sorted(self.sweep_samples, key=lambda s: -s[1])[:1]

        def score(sample):
            heading, dist = sample
            visits = self.visits_in_direction(heading)
            return (visits, -dist)

        best = min(candidates, key=score)
        self.target_theta = best[0]
        self.get_logger().info(
            f'[Explorer] chosen heading={math.degrees(self.target_theta):.0f} deg '
            f'dist={best[1]:.2f} m')
        self.state = State.TURN_TO_HEADING

    # ── TURN_TO_HEADING ────────────────────────
    def _do_turn_to_heading(self):
        err = wrap_angle(self.target_theta - self.theta)
        if abs(err) <= HEADING_TOLERANCE:
            self.publish_twist(0.0, 0.0)
            # ── NEW: reset leg-distance tracking at the start of each new leg ──
            self.leg_start_x = self.x
            self.leg_start_y = self.y
            self.state = State.DRIVE
            return
        direction = 1.0 if err > 0 else -1.0
        self.publish_twist(0.0, direction * TURN_SPEED)

    # ── RETURN_HOME ────────────────────────────
    def _do_return_home(self):
        dist_home = math.hypot(self.x, self.y)
        if dist_home <= RETURN_HOME_TOLERANCE:
            self.publish_twist(0.0, 0.0)
            self.get_logger().info('[Explorer] home reached — done')
            self.state = State.DONE
            return

        heading_home = math.atan2(-self.y, -self.x)
        err = wrap_angle(heading_home - self.theta)

        if self.front_dist < OBSTACLE_STOP_DIST:
            self.publish_twist(0.0, TURN_SPEED)
            return

        if abs(err) > HEADING_TOLERANCE:
            direction = 1.0 if err > 0 else -1.0
            self.publish_twist(0.0, direction * TURN_SPEED)
        else:
            self.publish_twist(FORWARD_SPEED, 0.0)


def main():
    rclpy.init()
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_twist(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()