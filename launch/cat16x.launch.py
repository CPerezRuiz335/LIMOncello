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
            PathJoinSubstitution([FindPackageShare('limoncello'), 'config', 'cat16x.yaml']),
            {'use_sim_time': False}
        ],
    )

    return LaunchDescription([
        limoncello_node
    ])
