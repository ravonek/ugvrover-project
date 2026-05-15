from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    roarm_node = Node(
        package='roarm_m2_s',
        executable='roarm',
        name='roarm',
        output='screen'
    )

    camera2_node = Node(
        package='roarm_m2_s',
        executable='camera2',
        name='camera2',
        output='screen'
    )

    pick_place_node = Node(
        package='roarm_m2_s',
        executable='pick_place',
        name='pick_place',
        output='screen'
    )

    return LaunchDescription([
        roarm_node,
        camera2_node,
        pick_place_node,
    ])
