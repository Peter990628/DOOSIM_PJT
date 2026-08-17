from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    tray_share = Path(get_package_share_directory("hospital_tray_overlay"))
    nav_share = Path(get_package_share_directory("hospital_nav2"))
    nav2_share = Path(get_package_share_directory("nav2_bringup"))
    params = tray_share / "config" / "backup_precoupled_nav2_guaranteed.yaml"
    map_yaml = nav_share / "maps" / "hospital_map.yaml"

    # map->coop_odom is started independently by RUN_BACKUP_2_GUARANTEED_V2_15.sh
    # so RViz/fallback remain usable even if the Nav2 launch process exits.

    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server",
        output="screen", parameters=[{"yaml_filename": str(map_yaml), "use_sim_time": True}],
    )
    map_lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="backup_guaranteed_map_lifecycle_manager", output="screen",
        parameters=[{"use_sim_time": True, "autostart": True, "node_names": ["map_server"]}],
    )

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2_share / "launch" / "navigation_launch.py")),
        launch_arguments={
            "namespace": "coopnav",
            "use_sim_time": "True",
            "params_file": str(params),
            "autostart": "True",
            "use_composition": "False",
            "use_respawn": "False",
        }.items(),
    )

    relay = Node(
        package="hospital_tray_overlay", executable="cooperative_cmd_vel_relay",
        namespace="coopnav", name="cooperative_cmd_vel_relay", output="screen",
        parameters=[{"input_topic": "/coopnav/cmd_vel", "output_topic": "/coop/cmd_vel"}],
    )

    return LaunchDescription([map_server, map_lifecycle, nav, relay])
