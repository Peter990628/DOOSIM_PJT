from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="hospital_tray_overlay",
            executable="tray_aruco_pair_node",
            name="amr1_tray_aruco_gate",
            output="screen",
            parameters=[{
                "amr_id": "amr1",
                "image_topic": "/amr1/camera/front/color/image_raw",
                "result_topic": "/amr1/tray_aruco/result",
                "debug_image_topic": "/amr1/tray_aruco/debug_image",
                "outer_ids": [40, 41],
                "center_id": 44,
                "outer_side": "left",
                "dictionary": "DICT_4X4_50",
                "publish_hz": 15.0,
                "show_window": True,
                "window_width": 760,
                "window_height": 520,
            }],
        ),
        Node(
            package="hospital_tray_overlay",
            executable="tray_aruco_pair_node",
            name="amr2_tray_aruco_gate",
            output="screen",
            parameters=[{
                "amr_id": "amr2",
                "image_topic": "/amr2/camera/front/color/image_raw",
                "result_topic": "/amr2/tray_aruco/result",
                "debug_image_topic": "/amr2/tray_aruco/debug_image",
                "outer_ids": [42, 43],
                "center_id": 44,
                "outer_side": "right",
                "dictionary": "DICT_4X4_50",
                "publish_hz": 15.0,
                "show_window": True,
                "window_width": 760,
                "window_height": 520,
            }],
        ),
    ])
