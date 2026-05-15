import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile

from nav2_common.launch import RewrittenYaml, ReplaceString


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    nav_pkg_dir = get_package_share_directory('ugv_navigation2')

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    use_rviz = LaunchConfiguration('use_rviz')

    rviz_config_file = os.path.join(
        nav_pkg_dir,
        'rviz',
        'view_nav_real.rviz'
    )

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static')
    ]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file
    }

    params_file = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': ('/', namespace)},
        condition=IfCondition(use_namespace)
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True
        ),
        allow_substs=True
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM',
        '1'
    )

    return LaunchDescription([
        stdout_linebuf_envvar,

        DeclareLaunchArgument(
            'namespace',
            default_value=''
        ),

        DeclareLaunchArgument(
            'use_namespace',
            default_value='false'
        ),

        DeclareLaunchArgument(
            'slam',
            default_value='False'
        ),

        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(nav_pkg_dir, 'map', 'my_map.yaml')
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                nav_pkg_dir,
                'param',
                'ugv_rover_real.yaml'
            )
        ),

        DeclareLaunchArgument(
            'autostart',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'use_composition',
            default_value='True'
        ),

        DeclareLaunchArgument(
            'use_respawn',
            default_value='False'
        ),

        DeclareLaunchArgument(
            'log_level',
            default_value='info'
        ),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true'
        ),

        GroupAction([
            PushRosNamespace(
                condition=IfCondition(use_namespace),
                namespace=namespace
            ),

            Node(
                condition=IfCondition(use_composition),
                name='nav2_container',
                package='rclcpp_components',
                executable='component_container_isolated',
                parameters=[
                    configured_params,
                    {'autostart': autostart}
                ],
                arguments=[
                    '--ros-args',
                    '--log-level',
                    log_level
                ],
                remappings=remappings,
                output='screen'
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'localization_launch.py')
                ),
                condition=IfCondition(PythonExpression(['not ', slam])),
                launch_arguments={
                    'namespace': namespace,
                    'map': map_yaml_file,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': params_file,
                    'use_composition': use_composition,
                    'use_respawn': use_respawn,
                    'container_name': 'nav2_container'
                }.items()
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'navigation_launch.py')
                ),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': params_file,
                    'use_composition': use_composition,
                    'use_respawn': use_respawn,
                    'container_name': 'nav2_container'
                }.items()
            ),
        ]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            condition=IfCondition(use_rviz)
        )
    ])
