from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('hospital_nav2'))
    nav2_share = Path(get_package_share_directory('nav2_bringup'))

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2_share / 'launch' / 'navigation_launch.py')),
        launch_arguments={
            'namespace': '',
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'True',
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('map', default_value=str(share / 'maps' / 'hospital_map.yaml')),
        DeclareLaunchArgument(
            'params_file',
            default_value=str(share / 'config' / 'nav2_params_trolley_smac.yaml'),
        ),
        # V4.3: Nav2/behavior outputs go to RAW. Segment-heading gate is final authority.
        SetRemap(src='/cmd_vel', dst='/trolley/cmd_vel_raw'),
        SetRemap(src='cmd_vel', dst='/trolley/cmd_vel_raw'),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file, {'yaml_filename': map_file, 'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server'],
                'bond_timeout': 0.0,
            }],
        ),
        # V4.6: global planner receives only the forward 160-degree LiDAR sector.
        # Local costmap still uses the full /trolley/scan for 360-degree safety.
        Node(
            package='hospital_nav2',
            executable='trolley_front_scan_filter',
            name='trolley_front_scan_filter',
            output='screen',
            parameters=[{
                'input_topic': '/trolley/scan',
                'output_topic': '/trolley/scan_front',
                'front_half_angle_deg': 80.0,
            }],
        ),
        navigation,
        # V4.6: final motion authority. Only confirmed cumulative corners force STOP+ROTATE.
        Node(
            package='hospital_nav2',
            executable='trolley_heading_gate',
            name='trolley_heading_gate',
            output='screen',
            parameters=[{
                'input_topic': '/trolley/cmd_vel_raw',
                'output_topic': '/trolley/cmd_vel',
                'plan_topic': '/plan',
                'global_frame': 'map',
                'base_frame': 'trolley_base',
                'dominant_segment_span_m': 0.80,
                'corner_detect_angle_deg': 35.0,
                'corner_onset_angle_deg': 12.0,
                'corner_scan_ahead_m': 3.0,
                'corner_target_lookahead_m': 0.90,
                'preturn_trigger_distance_m': 0.80,
                'corner_exit_angle_deg': 2.0,
                'consumed_corner_radius_m': 1.00,
                'drive_enter_angle_deg': 10.0,
                'drive_exit_angle_deg': 3.0,
                'drive_heading_kp': 1.0,
                'max_drive_angular_speed': 0.06,
                'rotate_kp': 1.6,
                'min_rotate_speed': 0.12,
                'max_rotate_speed': 0.35,
                'rotation_stall_time_sec': 1.0,
                'rotation_stall_yaw_deg': 0.5,
                'rotation_stall_boost_speed': 0.18,
                'replan_min_interval_sec': 2.0,
                'replan_lateral_threshold_m': 0.30,
                'replan_heading_threshold_deg': 15.0,
                'new_goal_endpoint_threshold_m': 0.40,
            }],
        ),
        # V4 Stage-1: keep the existing /trolley/center_goal interface, but do NOT
        # generate a hard-coded centerline. Forward the goal to NavigateToPose.
        Node(
            package='hospital_nav2',
            executable='trolley_goal_forwarder',
            name='trolley_goal_forwarder',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_trolley_smac',
            output='screen',
            arguments=['-d', str(share / 'rviz' / 'trolley_smac_navigation.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
