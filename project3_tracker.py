from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, Shutdown
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    node1 = Node(
        package='csce452_proj3',
        executable='lidar',
        name='lidar'
    )
    node2 = Node(
        package='csce452_proj3',
        executable='marker',
        name='marker'
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '/home/nohbody/.rviz2/project_3_config.rviz'],
    )
    arg1 = DeclareLaunchArgument(
        'bag_in',
        description='Path to the ros2 bag to play'
    )
    playback = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_in')]
    )

    #TODO: Add arguments and command to record a bag

    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=playback,
            on_exit=[Shutdown()]
        )
    )

    return LaunchDescription([
        node1,
        node2,
        rviz,
        arg1,
        playback,
        shutdown_on_exit
    ])