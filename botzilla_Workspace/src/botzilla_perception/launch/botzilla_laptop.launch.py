from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    weights_arg = DeclareLaunchArgument(
        'weights',
        default_value=os.path.join(
            os.path.dirname(__file__),
            '..', '..', '..', '..', '..', '..', '..', '..',  # repo root
            'runs', 'detect', 'train4', 'weights', 'best.pt'
        ),
        description='Path to YOLO best.pt weights file'
    )

    return LaunchDescription([
        weights_arg,

        # --- Webcam bridge (replaces Kinect on laptop) ---
        Node(
            package='botzilla_perception',
            executable='webcam_bridge',
            name='kinect_bridge',
            output='screen',
        ),

        # --- YOLO Detector ---
        Node(
            package='botzilla_perception',
            executable='yolo_node',
            name='yolo_node',
            output='screen',
            parameters=[{
                'weights':     LaunchConfiguration('weights'),
                'confidence':  0.5,
                'show_window': True,   # set False on Pi (headless)
            }]
        ),

        # --- Mock Kobuki (laptop only — no hardware) ---
        Node(
            package='botzilla_control',
            executable='mock_kobuki_node',
            name='kobuki_base_node',
            output='screen',
        ),
    ])
