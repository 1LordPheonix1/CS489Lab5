from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    pure_pursuit = Node(
        package='pure_pursuit',
        executable='pure_pursuit_node',
        name='pure_pursuit',
        output='screen',
        parameters=[]
    )

    logger = Node(
        package='pure_pursuit',
        executable='logger_node',
        name='logger',
        output='screen',
        parameters=[]
    )

    ld.add_action(pure_pursuit)
    # ld.add_action(logger)
    
    return ld
