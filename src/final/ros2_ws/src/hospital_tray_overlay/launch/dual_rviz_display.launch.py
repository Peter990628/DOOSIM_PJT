from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
  share=Path(get_package_share_directory("hospital_tray_overlay"))
  return LaunchDescription([
    Node(package="hospital_tray_overlay",executable="amr2_tf_relay",name="amr2_tf_display_relay",output="screen"),
    Node(package="rviz2",executable="rviz2",name="tray_dual_rviz_display",output="screen",arguments=["-d",str(share/"rviz"/"dual_navigation_display_only.rviz")],parameters=[{"use_sim_time":True}]),
  ])
