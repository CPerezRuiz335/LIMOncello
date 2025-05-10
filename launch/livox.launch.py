from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    limoncello_node = Node(
        package='limoncello',
        namespace='',
        executable='limoncello',
        name='limoncello',
        output='screen',
        parameters=[
            PathJoinSubstitution([FindPackageShare('limoncello'), 'config', 'livox.yaml']),
            {'use_sim_time': False}
        ]
    )

    return LaunchDescription([
        limoncello_node
    ])
