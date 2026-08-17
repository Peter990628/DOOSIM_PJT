from pathlib import Path
import json
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _select_lattice_file() -> str:
    """Use the primitive pinned by the run script, or deterministically select one.

    V5.7 never silently switches to an Ackermann primitive. The selected file is
    also JSON-parsed here so a corrupt/non-JSON path fails before Nav2 starts.
    """
    pinned = os.environ.get('HOSPITAL_LATTICE_FILE', '').strip()
    if pinned:
        path = Path(pinned)
        if not path.is_file():
            raise RuntimeError(f'HOSPITAL_LATTICE_FILE does not exist: {path}')
        if 'diff' not in str(path).lower():
            raise RuntimeError(f'Pinned lattice is not differential-drive: {path}')
        json.loads(path.read_text())
        return str(path)

    share = Path(get_package_share_directory('nav2_smac_planner'))
    jsons = [p for p in share.rglob('*.json') if 'diff' in str(p).lower()]
    if not jsons:
        raise RuntimeError(f'No differential-drive State Lattice JSON under {share}')

    def score(path: Path):
        t = str(path).lower()
        return (
            0 if ('5cm' in t or '0.05' in t or '5_cm' in t) else 1,
            0 if ('0.5m' in t or '0.5_m' in t or '0.5' in t) else 1,
            str(path),
        )

    path = sorted(jsons, key=score)[0]
    json.loads(path.read_text())
    return str(path)


def _make_runtime_params(src: Path, lattice_file: str) -> str:
    text = src.read_text()
    # Empty is explicitly supported by SmacPlannerLattice as the basic/default test lattice.
    text = text.replace('__LATTICE_FILE__', lattice_file)
    out = Path(tempfile.gettempdir()) / 'hospital_nav2_trolley_lattice_runtime.yaml'
    out.write_text(text)
    return str(out)


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('hospital_nav2'))
    nav2_share = Path(get_package_share_directory('nav2_bringup'))

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')

    lattice_file = _select_lattice_file()
    params_runtime = _make_runtime_params(
        share / 'config' / 'nav2_params_trolley_lattice.yaml', lattice_file
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2_share / 'launch' / 'navigation_launch.py')),
        launch_arguments={
            'namespace': '',
            'use_sim_time': use_sim_time,
            'params_file': params_runtime,
            'autostart': 'True',
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('map', default_value=str(share / 'maps' / 'hospital_map.yaml')),

        LogInfo(msg='[V5.13] CLEARANCE-DP + CORNER CENTERING + TRACK-ERROR REPLAN + MPPI'),
        LogInfo(msg=f'[V5.7] lattice primitive file: {lattice_file}'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_runtime, {'yaml_filename': map_file, 'use_sim_time': use_sim_time}],
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

        # Keep the V4.6 improvement: global planner sees only forward 160 deg.
        # Local costmap remains on full /trolley/scan for 360-degree safety.
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

        # Keep Isaac's external command API unchanged. Nav2 Humble produces its
        # final smoothed command on /cmd_vel; relay only the final command outward.
        Node(
            package='hospital_nav2',
            executable='trolley_cmd_vel_relay',
            name='trolley_cmd_vel_relay',
            output='screen',
        ),

        # V5.7: raw State-Lattice path is optimized before FollowPath.
        # Preserve lattice yaw, run DP L/R clearance + swept-footprint checks, then FollowPath.
        Node(
            package='hospital_nav2',
            executable='trolley_clearance_navigator',
            name='trolley_clearance_navigator',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'goal_topic': '/trolley/center_goal',
                'costmap_topic': '/global_costmap/costmap',
                'full_scan_topic': '/trolley/scan',
                'raw_path_topic': '/trolley/raw_plan',
                'optimized_path_topic': '/trolley/clearance_plan',
                'planner_id': 'GridBased',
                'global_frame': 'map',
                'base_frame': 'trolley_base',
                'trolley_half_length_m': 1.18,
                'trolley_half_width_m': 0.95,
                'side_scan_m': 2.30,
                'side_step_m': 0.05,
                'side_longitudinal_samples': 9,
                'max_lateral_shift_m': 1.20,
                'candidate_step_m': 0.10,
                'optimize_spacing_m': 0.20,
                'optimize_yaw_spacing_deg': 7.5,
                'max_shift_step_m': 0.15,
                'collision_cost_threshold': 100,
                'obstacle_cost_threshold': 90,
                'footprint_sample_step_m': 0.06,
                'swept_linear_step_m': 0.06,
                'swept_angular_step_deg': 4.0,
                'w_balance': 16.0,
                'w_min_clearance': 8.0,
                'w_inflation_sum': 3.0,
                'w_inflation_balance': 10.0,
                'w_deviation': 1.00,
                'w_shift_smooth': 3.0,
                'w_shift_accel': 4.0,
                'w_extra_path_length': 5.0,
                'corner_soften_yaw_deg': 12.0,
                'corner_disable_yaw_deg': 28.0,
                'corner_min_center_factor': 0.50,
                'corridor_narrow_clearance_m': 0.95,
                'corridor_open_clearance_m': 1.55,
                'corridor_center_gain': 2.40,
                'start_lock_m': 0.15,
                'goal_rotation_clearance_margin_m': 0.12,
                'goal_staging_min_m': 0.55,
                'goal_staging_max_m': 1.60,
                'goal_staging_step_m': 0.15,
                'goal_rotation_step_deg': 4.0,
                'enable_conditional_replan': True,
                'replan_check_period_sec': 0.40,
                'replan_cooldown_sec': 1.5,
                'replan_start_grace_sec': 2.0,
                'dynamic_obstacle_confirmations': 3,
                'replan_check_stride': 2,
                'replan_path_deviation_m': 0.25,
                'replan_deviation_confirmations': 2,
                'wall_replan_check_period_sec': 0.50,
                'wall_replan_confirmations': 3,
                'wall_replan_min_clearance_m': 0.30,
                'wall_replan_imbalance_m': 0.35,
                'forward_fov_deg': 170.0,
                'forward_sector_percentile': 10.0,
                'front_sector_clearance_m': 0.55,
                'front_diagonal_clearance_m': 0.40,
                'side_sector_clearance_m': 0.30,
                'edge_replan_min_clearance_m': 0.30,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_trolley_lattice',
            output='screen',
            arguments=['-d', str(share / 'rviz' / 'trolley_smac_navigation.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
