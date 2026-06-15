from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    TEST 6: Cube Following Test (Brain Node Part 1)
    =============================================
    Tests that the robot can search for a cube, align with it,
    and drive toward it until it is grabbed.
    """

    kobuki_base = Node(
        package='botzilla_control',
        executable='kobuki_base_node',
        name='kobuki_base_node',
        output='screen',
    )

    kinect_bridge = Node(
        package='botzilla_perception',
        executable='kinect_bridge',
        name='kinect_bridge',
        output='screen',
    )

    yolo_node = Node(
        package='botzilla_perception',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
    )

    brain_node = Node(
        package='botzilla_control',
        executable='brain_node',
        name='botzilla_brain',
        output='screen',
    )

    return LaunchDescription([
        kobuki_base,
        kinect_bridge,
        yolo_node,
        brain_node,
    ])
