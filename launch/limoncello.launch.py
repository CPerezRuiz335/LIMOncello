from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_name  = LaunchConfiguration('config_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz         = LaunchConfiguration('rviz')

    config_file = PathJoinSubstitution([
        FindPackageShare('limoncello'),
        'config',
        [config_name, TextSubstitution(text='.yaml')]
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('limoncello'), 'config', 'rviz', 'limoncello.rviz'
    ])

    limoncello_node = Node(
        package='limoncello',
        namespace='limoncello',
        executable='limoncello',
        name='slam',
        output='screen',
        emulate_tty=True,
        parameters=[config_file, {'use_sim_time': use_sim_time}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_name',
            description='YAML config basename inside limoncello/config'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz'
        ),
        limoncello_node,
        rviz_node,
    ])