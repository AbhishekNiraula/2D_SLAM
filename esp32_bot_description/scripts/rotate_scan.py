#!/usr/bin/env python3
# ============================================================
#  rotate_scan.py — stationary rotation + custom point-cloud
#                   mapper, saves a PNG map on shutdown
#
#  Bot stays fixed at its placed position and rotates very
#  slowly in place. Each valid /scan reading is converted to a
#  world-frame (x, y) point using the live /odom pose and
#  accumulated into a hit list. On Ctrl+C (or when the safety
#  rotation cap is hit) the accumulated points are rendered and
#  saved as a PNG — no slam_toolbox involved.
# ============================================================

import math
import os
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import matplotlib
matplotlib.use('Agg')  # no display needed, just render to file
import matplotlib.pyplot as plt


ROTATE_SPEED       = 0.05   # rad/s — very slow, odometry/scan can keep up
CMD_PUBLISH_HZ     = 10.0   # must stay well under firmware's 500 ms watchdog
MAX_ROTATIONS      = 50     # safety cap; Ctrl+C manually well before this
MIN_ANGLE_STEP     = math.radians(1.0)   # only record a new point every ~1 deg
RANGE_MAX_EPSILON  = 0.02   # m — treat readings within this of range_max as "no hit"
FIRMWARE_RANGE_MAX = 2.0    # must match scan_msg.range_max in main.cpp

AUTOSAVE_EVERY_S   = 30.0   # periodic safety save in case of crash
OUTPUT_DIR         = os.path.expanduser('~/slam_bot_maps')
OUTPUT_FILE        = 'map.png'


def wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class RotateScanMapper(Node):
    def __init__(self):
        super().__init__('rotate_scan_mapper')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        self.have_odom = False
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.start_x = None
        self.start_y = None

        self._last_theta = None
        self._accum_rotation = 0.0
        self._rotations_done = 0

        self._last_recorded_theta = None
        self.points = []  # list of (x, y) world-frame hit points

        self._last_autosave = time.time()

        os.makedirs(OUTPUT_DIR, exist_ok=True)

        self.timer = self.create_timer(1.0 / CMD_PUBLISH_HZ, self.tick)
        self.get_logger().info(
            '[RotateScanMapper] started — rotating slowly, Ctrl+C to stop and save map')

    # ─────────────────────────────────────────
    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = 2.0 * math.atan2(q.z, q.w)
        self.have_odom = True

        if self.start_x is None:
            self.start_x = self.x
            self.start_y = self.y

        if self._last_theta is None:
            self._last_theta = self.theta
        else:
            d = wrap_angle(self.theta - self._last_theta)
            self._accum_rotation += abs(d)
            self._last_theta = self.theta

        if self._accum_rotation >= 2.0 * math.pi:
            self._accum_rotation -= 2.0 * math.pi
            self._rotations_done += 1
            self.get_logger().info(
                f'[RotateScanMapper] completed rotation #{self._rotations_done} '
                f'— {len(self.points)} points so far')
            self.save_map()  # save at the end of every full rotation too
            if self._rotations_done >= MAX_ROTATIONS:
                self.get_logger().info('[RotateScanMapper] safety cap reached — stopping')
                self.shutdown_and_save()

        if (time.time() - self._last_autosave) > AUTOSAVE_EVERY_S:
            self._last_autosave = time.time()
            self.save_map()

    # ─────────────────────────────────────────
    def scan_cb(self, msg: LaserScan):
        if not self.have_odom:
            return
        valid = [r for r in msg.ranges if r > 0.0]
        if not valid:
            return
        dist = min(valid)

        # skip "no obstacle detected" readings (firmware reports range_max)
        if dist >= (FIRMWARE_RANGE_MAX - RANGE_MAX_EPSILON):
            return

        # only record roughly every MIN_ANGLE_STEP of rotation, to avoid
        # dozens of near-duplicate points piling up at the same heading
        if self._last_recorded_theta is not None:
            if abs(wrap_angle(self.theta - self._last_recorded_theta)) < MIN_ANGLE_STEP:
                return
        self._last_recorded_theta = self.theta

        hit_x = self.x + dist * math.cos(self.theta)
        hit_y = self.y + dist * math.sin(self.theta)
        self.points.append((hit_x, hit_y))

    # ─────────────────────────────────────────
    def publish_twist(self, linear, angular):
        t = Twist()
        t.linear.x = linear
        t.angular.z = angular
        self.cmd_pub.publish(t)

    def tick(self):
        if not self.have_odom:
            self.publish_twist(0.0, 0.0)
            return
        self.publish_twist(0.0, ROTATE_SPEED)

    # ─────────────────────────────────────────
    def save_map(self):
        if not self.points:
            self.get_logger().info('[RotateScanMapper] no points yet — skipping save')
            return

        xs = [p[0] for p in self.points]
        ys = [p[1] for p in self.points]

        fig, ax = plt.subplots(figsize=(8, 8))
        ax.scatter(xs, ys, s=4, c='black', label='detected wall points')
        if self.start_x is not None:
            ax.scatter([self.start_x], [self.start_y], s=80, c='red',
                       marker='o', label='robot position', zorder=5)
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_title(f'SLAM_Bot map — {len(self.points)} points, '
                     f'{self._rotations_done} full rotations')
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, linestyle='--', alpha=0.3)

        out_path = os.path.join(OUTPUT_DIR, OUTPUT_FILE)
        fig.savefig(out_path, dpi=200, bbox_inches='tight')
        plt.close(fig)
        self.get_logger().info(f'[RotateScanMapper] map saved to {out_path}')

    def shutdown_and_save(self):
        self.publish_twist(0.0, 0.0)
        self.save_map()


def main():
    rclpy.init()
    node = RotateScanMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_and_save()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()