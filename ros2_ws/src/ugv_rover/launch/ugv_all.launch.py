from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    cmd_in_topic_arg = DeclareLaunchArgument(
        'cmd_in_topic',
        default_value='/cmd_vel_sim'
    )

    cmd_real_topic_arg = DeclareLaunchArgument(
        'cmd_real_topic',
        default_value='/cmd_vel'
    )

    odom_sim_topic_arg = DeclareLaunchArgument(
        'odom_sim_topic',
        default_value='/odom_sim'
    )

    odom_real_topic_arg = DeclareLaunchArgument(
        'odom_real_topic',
        default_value='/odom'
    )

    real_linear_gain_arg = DeclareLaunchArgument(
        'real_linear_gain',
        default_value='0.25'
    )

    real_angular_gain_arg = DeclareLaunchArgument(
        'real_angular_gain',
        default_value='0.30'
    )

    kp_linear_arg = DeclareLaunchArgument(
        'kp_linear',
        default_value='0.008'
    )

    kp_angular_arg = DeclareLaunchArgument(
        'kp_angular',
        default_value='0.008'
    )

    max_linear_arg = DeclareLaunchArgument(
        'max_linear',
        default_value='0.22'
    )

    max_angular_arg = DeclareLaunchArgument(
        'max_angular',
        default_value='0.55'
    )

    nav_goal_node = Node(
        package='ugv_rover',
        executable='nav_goal',
        name='nav_goal',
        output='screen'
    )

    sim_real_sync_guard_node = Node(
        package='ugv_rover',
        executable='sim_real_sync_guard',
        name='sim_real_sync_guard',
        output='screen',
        parameters=[{
            'cmd_in_topic': LaunchConfiguration('cmd_in_topic'),
            'cmd_real_topic': LaunchConfiguration('cmd_real_topic'),
            'odom_sim_topic': LaunchConfiguration('odom_sim_topic'),
            'odom_real_topic': LaunchConfiguration('odom_real_topic'),
            'real_linear_gain': LaunchConfiguration('real_linear_gain'),
            'real_angular_gain': LaunchConfiguration('real_angular_gain'),
            'kp_linear': LaunchConfiguration('kp_linear'),
            'kp_angular': LaunchConfiguration('kp_angular'),
            'max_linear': LaunchConfiguration('max_linear'),
            'max_angular': LaunchConfiguration('max_angular'),
        }]
    )

    cube_node = Node(
        package='ugv_rover',
        executable='cube',
        name='cube_joint_controller',
        output='screen'
    )

    return LaunchDescription([
        cmd_in_topic_arg,
        cmd_real_topic_arg,
        odom_sim_topic_arg,
        odom_real_topic_arg,
        real_linear_gain_arg,
        real_angular_gain_arg,
        kp_linear_arg,
        kp_angular_arg,
        max_linear_arg,
        max_angular_arg,

        nav_goal_node,
        sim_real_sync_guard_node,
        cube_node,
    ])
