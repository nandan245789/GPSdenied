"""Full system launch — all subsystems for simulation or hardware."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim = DeclareLaunchArgument('use_sim', default_value='true')

    return LaunchDescription([
        use_sim,

        # Perception
        Node(package='gps_denied_perception', executable='feature_extractor',
             name='feature_extractor', output='screen'),
        Node(package='gps_denied_perception', executable='depth_processor',
             name='depth_processor', output='screen'),

        # State Estimation
        Node(package='gps_denied_state_estimation', executable='vio_node',
             name='vio_node', output='screen'),
        Node(package='gps_denied_state_estimation', executable='px4_vision_bridge',
             name='px4_vision_bridge', output='screen'),

        # Mapping
        Node(package='gps_denied_mapping', executable='octomap_builder',
             name='octomap_builder', output='screen'),

        # Planning
        Node(package='gps_denied_planning', executable='waypoint_manager',
             name='waypoint_manager', output='screen'),
        Node(package='gps_denied_planning', executable='local_planner',
             name='local_planner', output='screen'),

        # Control
        Node(package='gps_denied_control', executable='trajectory_tracker',
             name='trajectory_tracker', output='screen'),
        Node(package='gps_denied_control', executable='px4_commander',
             name='px4_commander', output='screen'),

        # Safety
        Node(package='gps_denied_safety', executable='mission_supervisor',
             name='mission_supervisor', output='screen'),

        # Telemetry
        Node(package='gps_denied_bringup', executable='telemetry_aggregator',
             name='telemetry_aggregator', output='screen'),
    ])
