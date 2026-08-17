from glob import glob
from setuptools import find_packages, setup

package_name = "hospital_nav2"

setup(
    name=package_name,
    version="2.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/maps", glob("maps/*")),
        ("share/" + package_name + "/urdf", glob("urdf/*.urdf")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="rokey",
    maintainer_email="rokey@example.com",
    description="Hospital AMR pose-lock localization and corridor-center navigation.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pose_lock_localizer = hospital_nav2.pose_lock_localizer:main",
            "centerline_navigator = hospital_nav2.centerline_navigator:main",
            "trolley_goal_forwarder = hospital_nav2.trolley_goal_forwarder:main",
            "trolley_heading_gate = hospital_nav2.trolley_heading_gate:main",
            "trolley_front_scan_filter = hospital_nav2.trolley_front_scan_filter:main",
            "trolley_clearance_navigator = hospital_nav2.trolley_clearance_navigator:main",
            "trolley_cmd_vel_relay = hospital_nav2.trolley_cmd_vel_relay:main",
            "world_pose_initializer = hospital_nav2.world_pose_initializer:main",
            "corridor_priority_manager = hospital_nav2.corridor_priority_manager:main",
        ],
    },
)
