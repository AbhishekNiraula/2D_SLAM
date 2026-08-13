#!/usr/bin/env python3
# ============================================================
#  stepped_rotate.py — stepped in-place rotation, movement only
#
#  Turns a small fixed step, stops completely, pauses so
#  slam_toolbox gets a settled, well-timed /scan + /tf pair to
#  match against, then repeats. No custom point recording here —
#  slam_toolbox builds the occupancy grid from /scan and /tf as
#  usual. Run alongside slam_toolbox; save the resulting map with
#  nav2_map_server's map_saver_cli once this node reports done.
# ============================================================

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


STEP_ANGLE           = math.radians(5.0)   # rotate this much per step
TURN_SPEED            = 0.15   # rad/s while actively turning between steps
PAUSE_TICKS           = 12     # ticks to hold still after each step (see CMD_PUBLISH_HZ)
CMD_PUBLISH_HZ        = 10.0   # must stay under firmware's 500 ms watchdog

FULL_ROTATION         = 2.0 * math.pi
ROTATION_DONE_MARGIN  = math.radians(3.0)


def wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class State:
    INIT = 'INIT'
    TURN = 'TURN'
    PAUSE = 'PAUSE'
    DONE = 'DONE'


class SteppedRotate(Node):
    def __init__(self):
        super().__init__('stepped_rotate')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        self.have_odom = False
        self.theta = 0.0

        self._accum_rotation = 0.0
        self._last_theta_for_accum = None
        self._step_start_rotation = 0.0

        self.state = State.INIT
        self._pause_ticks_left = 0
        self._steps_done = 0

        self.timer = self.create_timer(1.0 / CMD_PUBLISH_HZ, self.tick)
        self.get_logger().info(
            '[SteppedRotate] started — will complete one stepped rotation, '
            'let slam_toolbox build the map, then stop')

    def odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.theta = 2.0 * math.atan2(q.z, q.w)

        if not self.have_odom:
            self.have_odom = True
            self._last_theta_for_accum = self.theta
            self.state = State.TURN
            self._step_start_rotation = 0.0
            self.get_logger().info('[SteppedRotate] odom acquired — starting sweep')
            return

        d = wrap_angle(self.theta - self._last_theta_for_accum)
        self._accum_rotation += abs(d)
        self._last_theta_for_accum = self.theta

    def publish_twist(self, linear, angular):
        t = Twist()
        t.linear.x = linear
        t.angular.z = angular
        self.cmd_pub.publish(t)

    def tick(self):
        if self.state == State.INIT:
            self.publish_twist(0.0, 0.0)
        elif self.state == State.TURN:
            self._do_turn()
        elif self.state == State.PAUSE:
            self._do_pause()
        elif self.state == State.DONE:
            self.publish_twist(0.0, 0.0)

    def _do_turn(self):
        turned_this_step = self._accum_rotation - self._step_start_rotation
        if turned_this_step >= STEP_ANGLE:
            self.publish_twist(0.0, 0.0)
            self._pause_ticks_left = PAUSE_TICKS
            self.state = State.PAUSE
            return
        self.publish_twist(0.0, TURN_SPEED)

    def _do_pause(self):
        self.publish_twist(0.0, 0.0)
        self._pause_ticks_left -= 1
        if self._pause_ticks_left > 0:
            return

        self._steps_done += 1
        self._step_start_rotation = self._accum_rotation
        self.get_logger().info(
            f'[SteppedRotate] step {self._steps_done} done — '
            f'{math.degrees(self._accum_rotation):.0f}/360 deg turned')

        if self._accum_rotation >= (FULL_ROTATION - ROTATION_DONE_MARGIN):
            self.get_logger().info(
                '[SteppedRotate] full rotation complete — stopping. '
                'Save the map now with map_saver_cli.')
            self.publish_twist(0.0, 0.0)
            self.state = State.DONE
            return

        self.state = State.TURN


def main():
    rclpy.init()
    node = SteppedRotate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_twist(0.0, 0.0)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()