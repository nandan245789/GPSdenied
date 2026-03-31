"""Phase 1 test launch — State estimation nodes only.

Launches VIO + PX4 bridge + ground truth comparator.
Use with PX4 SITL already running.

Usage:
  ros2 launch gps_denied_bringup test_state_estimation.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # VIO — visual-inertial odometry
        Node(
            package='gps_denied_state_estimation',
            executable='vio_node',
            name='vio_node',
            output='screen',
            parameters=[{
                'feature_threshold': 30,
                'max_features': 300,
                'min_feature_distance': 15.0,
                'confidence_threshold': 0.3,
                'publish_tf': True,
            }],
        ),

        # PX4 vision bridge — forward VIO to PX4 EKF2
        Node(
            package='gps_denied_state_estimation',
            executable='px4_vision_bridge',
            name='px4_vision_bridge',
            output='screen',
        ),

        # Ground truth comparator (sim only)
        Node(
            package='gps_denied_state_estimation',
            executable='ground_truth_comparator',
            name='ground_truth_comparator',
            output='screen',
            parameters=[{
                'gt_topic': '/model/x500/pose',
                'report_interval': 5.0,
            }],
        ),
    ])
