from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    limoncello_node = Node(
        package='limoncello',
        namespace='limoncello',
        executable='limoncello',
        name='slam',
        output='screen',
        emulate_tty=True,
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('limoncello'),
                'config',
                'long_corridor.yaml'
            ]),
            {'use_sim_time': True}
        ],
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare('limoncello'),
        'config',
        'rviz',
        'limoncello.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        limoncello_node,
        rviz_node
    ])