from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav_share=get_package_share_directory('hospital_nav2')
    map_yaml=os.path.join(nav_share,'maps','hospital_map.yaml')
    return LaunchDescription([
        Node(package='nav2_map_server',executable='map_server',name='map_server',output='screen',parameters=[{'yaml_filename':map_yaml,'use_sim_time':True}]),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='v216_map_lifecycle_manager',output='screen',parameters=[{'autostart':True,'node_names':['map_server'],'use_sim_time':True}]),
    ])
