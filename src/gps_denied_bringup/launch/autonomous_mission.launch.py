"""Full autonomous mission launch — all nodes for GPS-denied waypoint navigation.

This launches the complete pipeline:
  Camera → VIO → Map → Planner → Control → PX4

Usage:
  ros2 launch gps_denied_bringup autonomous_mission.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # --- Perception ---
        Node(
            package='gps_denied_perception',
            executable='depth_processor',
            name='depth_processor',
            output='screen',
            parameters=[{
                'max_range': 5.0,
                'min_range': 0.3,
                'voxel_size': 0.05,
                'downsample_factor': 2,
            }],
        ),

        # --- State Estimation ---
        Node(
            package='gps_denied_state_estimation',
            executable='vio_node',
            name='vio_node',
            output='screen',
            parameters=[{
                'feature_threshold': 30,
                'max_features': 300,
                'confidence_threshold': 0.3,
            }],
        ),
        Node(
            package='gps_denied_state_estimation',
            executable='px4_vision_bridge',
            name='px4_vision_bridge',
            output='screen',
        ),

        # --- Mapping ---
        Node(
            package='gps_denied_mapping',
            executable='octomap_builder',
            name='octomap_builder',
            output='screen',
            parameters=[{
                'resolution': 0.10,
                'local_map_radius': 8.0,
                'obstacle_inflation': 0.3,
                'publish_rate': 5.0,
            }],
        ),

        # --- Planning (start after 3s to let VIO initialize) ---
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='gps_denied_planning',
                    executable='waypoint_manager',
                    name='waypoint_manager',
                    output='screen',
                    parameters=[{
                        'mission_file': '/workspace/missions/warehouse_inspection.yaml',
                        'arrival_tolerance': 0.5,
                    }],
                ),
                Node(
                    package='gps_denied_planning',
                    executable='local_planner',
                    name='local_planner',
                    output='screen',
                    parameters=[{
                        'replan_rate': 2.0,
                        'max_velocity': 1.5,
                        'rrt_max_iter': 500,
                        'safety_margin': 0.4,
                    }],
                ),
            ],
        ),

        # --- Control ---
        Node(
            package='gps_denied_control',
            executable='trajectory_tracker',
            name='trajectory_tracker',
            output='screen',
            parameters=[{
                'control_rate': 50.0,
                'max_velocity': 1.5,
                'position_kp': 1.0,
            }],
        ),

        # --- Safety ---
        Node(
            package='gps_denied_safety',
            executable='mission_supervisor',
            name='mission_supervisor',
            output='screen',
        ),

        # --- Telemetry ---
        Node(
            package='gps_denied_state_estimation',
            executable='ground_truth_comparator',
            name='ground_truth_comparator',
            output='screen',
        ),
    ])
