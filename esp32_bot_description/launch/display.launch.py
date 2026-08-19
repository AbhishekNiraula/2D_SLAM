import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir   = get_package_share_directory('esp32_bot')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'esp32_bot.urdf')
    rviz_file = os.path.join(pkg_dir, 'rviz', 'esp32_bot.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        # ── 1. robot_state_publisher ──────────────────────────────
        # Publishes the full TF tree from the URDF, including
        # base_link → wheel_left, wheel_right, caster_front,
        # caster_rear, and laser.
        Node(
            package    = 'robot_state_publisher',
            executable = 'robot_state_publisher',
            name       = 'robot_state_publisher',
            output     = 'screen',
            parameters = [{'robot_description': robot_description,
                           'use_sim_time': False}],
        ),

        # ── 2. joint_state_publisher ──────────────────────────────
        # Publishes /joint_states so wheels animate in RViz.
        Node(
            package    = 'joint_state_publisher',
            executable = 'joint_state_publisher',
            name       = 'joint_state_publisher',
            output     = 'screen',
            parameters = [{'use_sim_time': False}],
        ),

        # ── 3. tf_relay ───────────────────────────────────────────
        # Bridges /tf_raw (TransformStamped from ESP32 odom→base_link)
        # → /tf (TFMessage for RViz2 + SLAM toolbox)
        Node(
            package    = 'esp32_bot',
            executable = 'tf_relay',
            name       = 'tf_relay',
            output     = 'screen',
        ),

        # ── 4. Persistent ToF occupancy-grid mapper ──────────────
        # Converts /scan + /odom into a persistent /map topic.
        Node(
            package    = 'esp32_bot',
            executable = 'tof_mapper',
            name       = 'tof_mapper',
            output     = 'screen',
        ),

        # ── 5. Slow right-hand wall follower ────────────────────
        # Publishes /cmd_vel after valid /scan data arrives. The
        # physical motor-enable button remains the final safety gate.
        Node(
            package    = 'esp32_bot',
            executable = 'wall_follower',
            name       = 'wall_follower',
            output     = 'screen',
        ),

        # ── 6. RViz2 ──────────────────────────────────────────────
        Node(
            package    = 'rviz2',
            executable = 'rviz2',
            name       = 'rviz2',
            output     = 'screen',
            arguments  = ['-d', rviz_file],
        ),
        
        # Explorer is disabled because it publishes /cmd_vel itself.
        # Use teleop_twist_keyboard to drive while the mapper records data.
        # Node(
		# 	package    = 'esp32_bot',
		# 	executable = 'rotate_scan',
		# 	name       = 'rotate_scan',
		# 	output     = 'screen',
		# ),
        # Node(
		# 	package    = 'esp32_bot',
		# 	executable = 'stepped_rotate',
		# 	name       = 'stepped_rotate',
		# 	output     = 'screen',
		# ),
    ])
