from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='csce452_proj3',
            executable='lidar',
            name='lidar'
        ),
        Node(
            package='csce452_proj3',
            executable='marker',
            name='marker'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/nohbody/.rviz2/project_3_config.rviz'],
        ),
        DeclareLaunchArgument(
            'bag_path',
            description='Path to the ros2 bag to play'
        ),
        #TODO: add argument to direct the output for the bag recordings for later
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_path')]
        )
    ])