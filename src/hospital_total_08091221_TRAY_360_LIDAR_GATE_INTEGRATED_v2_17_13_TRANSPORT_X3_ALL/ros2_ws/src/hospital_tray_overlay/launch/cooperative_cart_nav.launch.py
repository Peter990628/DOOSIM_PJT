from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    share=Path(get_package_share_directory("hospital_tray_overlay")); nav2_share=Path(get_package_share_directory("nav2_bringup"))
    params=share/"config"/"cooperative_nav2.yaml"
    parsed=yaml.safe_load(params.read_text())
    pose=dict(parsed["pose_lock_localizer"]["ros__parameters"]); center=dict(parsed["centerline_navigator"]["ros__parameters"])
    nav=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav2_share/"launch"/"navigation_launch.py")),launch_arguments={"namespace":"coopnav","use_sim_time":"True","params_file":str(params),"autostart":"True","use_composition":"False","use_respawn":"False"}.items())
    return LaunchDescription([
      Node(package="hospital_nav2",executable="pose_lock_localizer",namespace="coopnav",name="pose_lock_localizer",output="screen",parameters=[pose],remappings=[("/initialpose","/coopnav/initialpose"),("/initial_pose_locked","/coopnav/initial_pose_locked")]),
      nav,
      Node(package="hospital_tray_overlay",executable="cooperative_cmd_vel_relay",namespace="coopnav",name="cooperative_cmd_vel_relay",output="screen",parameters=[{"input_topic":"/coopnav/cmd_vel","output_topic":"/coop/cmd_vel"}]),
      Node(package="hospital_nav2",executable="centerline_navigator",namespace="coopnav",name="centerline_navigator",output="screen",parameters=[center]),
    ])
