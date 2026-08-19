#!/usr/bin/env python3
"""Build a persistent occupancy-grid map from the ESP32 ToF /scan topic.

The node uses /odom for the robot pose and publishes nav_msgs/OccupancyGrid
on /map.  It is intentionally lightweight: it does not perform scan matching
or pose correction, so map accuracy depends on the wheel odometry.
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan


UNKNOWN = -1
FREE = 0
OCCUPIED = 100


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class TofMapper(Node):
    def __init__(self):
        super().__init__('tof_mapper')

        # The map is centred around the robot's initial location in odom.
        self.declare_parameter('resolution', 0.05)       # metres/cell
        self.declare_parameter('width_m', 10.0)
        self.declare_parameter('height_m', 10.0)
        self.declare_parameter('laser_x', 0.0)           # laser offset in base_link
        self.declare_parameter('laser_y', 0.0)
        self.declare_parameter('laser_yaw', 0.0)         # radians
        self.declare_parameter('range_change_epsilon', 0.0005)  # metres
        # If the robot's pose has moved more than this since the last time
        # a bin was reprocessed, force full reprocessing of every valid bin
        # in the next scan regardless of whether its raw value happened to
        # change. Prevents a bin whose new (post-move) reading coincidentally
        # lands within range_change_epsilon of its pre-move value from
        # keeping a stale, wrongly-positioned map entry.
        self.declare_parameter('pose_change_distance_m', 0.01)
        self.declare_parameter('pose_change_heading_rad', 0.02)

        self.resolution = float(self.get_parameter('resolution').value)
        self.width_m = float(self.get_parameter('width_m').value)
        self.height_m = float(self.get_parameter('height_m').value)
        self.laser_x = float(self.get_parameter('laser_x').value)
        self.laser_y = float(self.get_parameter('laser_y').value)
        self.laser_yaw = float(self.get_parameter('laser_yaw').value)
        self.range_change_epsilon = float(
            self.get_parameter('range_change_epsilon').value)
        self.pose_change_distance_m = float(
            self.get_parameter('pose_change_distance_m').value)
        self.pose_change_heading_rad = float(
            self.get_parameter('pose_change_heading_rad').value)

        self.width = int(round(self.width_m / self.resolution))
        self.height = int(round(self.height_m / self.resolution))
        if self.width <= 0 or self.height <= 0 or self.resolution <= 0.0:
            raise ValueError('Map width, height, and resolution must be positive')

        self.origin_x = -self.width_m / 2.0
        self.origin_y = -self.height_m / 2.0
        self.cells = [UNKNOWN] * (self.width * self.height)

        self.have_odom = False
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.beams_written = 0
        self.previous_ranges = None
        self.previous_scan_geometry = None

        # Pose at the time the last scan was actually reprocessed into the
        # map (not just the latest /odom sample — see scan_callback).
        self.last_processed_x = None
        self.last_processed_y = None
        self.last_processed_yaw = None

        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 1)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 20)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_timer(1.0, self.publish_map)

        self.get_logger().info(
            f'ToF mapper ready: {self.width_m:.1f} x {self.height_m:.1f} m, '
            f'{self.resolution:.2f} m/cell. Publishing /map in the odom frame.')

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.have_odom = True

    def pose_moved_since_last_processed(self) -> bool:
        if self.last_processed_x is None:
            return True
        moved_distance = math.hypot(
            self.robot_x - self.last_processed_x,
            self.robot_y - self.last_processed_y,
        )
        moved_heading = abs(wrap_angle(self.robot_yaw - self.last_processed_yaw))
        return (moved_distance >= self.pose_change_distance_m or
                moved_heading >= self.pose_change_heading_rad)

    def scan_callback(self, msg: LaserScan):
        if not self.have_odom:
            return

        geometry = (msg.angle_min, msg.angle_increment, len(msg.ranges))
        geometry_changed = geometry != self.previous_scan_geometry
        if geometry_changed:
            self.previous_scan_geometry = geometry
            self.previous_ranges = None

        # If the robot's pose has moved since we last actually wrote data
        # into the map, every valid bin in this message must be treated as
        # fresh — the raw-value-delta check below can't tell the difference
        # between "sensor noise while stationary" and "genuinely new
        # geometry because the robot moved", so pose is checked first and
        # takes priority.
        if self.pose_moved_since_last_processed():
            self.previous_ranges = None

        # The ESP32 publishes a persistent servo buffer. Only integrate bins
        # that changed since the previous message; unchanged bins are old
        # measurements and must not be transformed using the new robot pose.
        changed_indices = []
        if self.previous_ranges is None:
            changed_indices = list(range(len(msg.ranges)))
        else:
            for index, distance in enumerate(msg.ranges):
                old_distance = self.previous_ranges[index]
                if self.range_changed(old_distance, distance):
                    changed_indices.append(index)
        self.previous_ranges = list(msg.ranges)

        if not changed_indices:
            return

        # Transform the laser mounting offset from base_link into odom.
        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)
        laser_x = self.robot_x + self.laser_x * cos_yaw - self.laser_y * sin_yaw
        laser_y = self.robot_y + self.laser_x * sin_yaw + self.laser_y * cos_yaw

        start = self.world_to_cell(laser_x, laser_y)
        if start is None:
            return

        for index in changed_indices:
            distance = msg.ranges[index]
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue

            angle = msg.angle_min + index * msg.angle_increment
            world_angle = self.robot_yaw + self.laser_yaw + angle
            hit_x = laser_x + distance * math.cos(world_angle)
            hit_y = laser_y + distance * math.sin(world_angle)
            end = self.world_to_cell(hit_x, hit_y)
            if end is None:
                continue

            # A valid distance clears cells along the beam and occupies its end.
            self.mark_free_ray(start, end)
            self.set_cell(end[0], end[1], OCCUPIED)
            self.beams_written += 1

        self.last_processed_x = self.robot_x
        self.last_processed_y = self.robot_y
        self.last_processed_yaw = self.robot_yaw

        self.publish_map()

    def range_changed(self, old_distance: float, new_distance: float) -> bool:
        old_valid = math.isfinite(old_distance)
        new_valid = math.isfinite(new_distance)
        if old_valid != new_valid:
            return True
        if not old_valid:
            return False
        return abs(new_distance - old_distance) >= self.range_change_epsilon

    def world_to_cell(self, x: float, y: float):
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        if 0 <= col < self.width and 0 <= row < self.height:
            return col, row
        return None

    def set_cell(self, col: int, row: int, value: int):
        self.cells[row * self.width + col] = value

    def mark_free_ray(self, start, end):
        """Bresenham line; leave the endpoint for the obstacle update."""
        x0, y0 = start
        x1, y1 = end
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        error = dx + dy

        while (x0, y0) != (x1, y1):
            cell_index = y0 * self.width + x0
            # A current valid free ray is allowed to clear an old obstacle.
            # This lets revisited areas be corrected when earlier readings
            # were noisy or stale.
            self.cells[cell_index] = FREE

            twice_error = 2 * error
            if twice_error >= dy:
                error += dy
                x0 += sx
            if twice_error <= dx:
                error += dx
                y0 += sy

    def publish_map(self):
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        # The ESP32 pose is published in odom, so the map is in that frame too.
        map_msg.header.frame_id = 'odom'
        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.width
        map_msg.info.height = self.height
        map_msg.info.origin.position.x = self.origin_x
        map_msg.info.origin.position.y = self.origin_y
        map_msg.info.origin.orientation.w = 1.0
        map_msg.data = self.cells
        self.map_pub.publish(map_msg)


def main():
    rclpy.init()
    node = TofMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_map()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()