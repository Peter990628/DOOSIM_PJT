from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("hospital_nav2"))
    nav2_share = Path(get_package_share_directory("nav2_bringup"))

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2_share / "launch" / "navigation_launch.py")),
        launch_arguments={
            "namespace": "",
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": "True",
            "use_composition": "False",
            "use_respawn": "False",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("map", default_value=str(share / "maps" / "hospital_map.yaml")),
            DeclareLaunchArgument(
                "params_file", default_value=str(share / "config" / "nav2_params_trolley.yaml")
            ),
            # Standard Nav2 nodes publish cmd_vel; remap that transport command
            # to the dedicated cooperative-trolley interface.
            SetRemap(src="/cmd_vel", dst="/trolley/cmd_vel"),
            SetRemap(src="cmd_vel", dst="/trolley/cmd_vel"),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[params_file, {"yaml_filename": map_file, "use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": ["map_server"],
                        "bond_timeout": 0.0,
                    }
                ],
            ),
            navigation,
            # Canonical goal path: RViz or a central manager publishes
            # /trolley/center_goal, then this node generates the corridor-centered
            # path and sends it to Nav2 FollowPath.
            Node(
                package="hospital_nav2",
                executable="centerline_navigator",
                name="centerline_navigator_trolley",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_trolley",
                output="screen",
                arguments=["-d", str(share / "rviz" / "trolley_navigation.rviz")],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
