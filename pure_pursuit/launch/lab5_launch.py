from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    file = LaunchConfiguration('file')
    lookahead = LaunchConfiguration('la')
    basespeed = LaunchConfiguration('speed')
    mxangle = LaunchConfiguration('mxangle')

    file_launch_arg = DeclareLaunchArgument(
        'file',
        default_value='/sim_ws/src/pure_pursuit/src/waypoints.csv'
    )
    
    lookahead_launch_arg = DeclareLaunchArgument(
        'la',
        default_value='1.0'
    )
    
    basespeed_launch_arg = DeclareLaunchArgument(
        'speed',
        default_value='1.0'
    )

    mxangle_launch_arg = DeclareLaunchArgument(
        'mxangle',
        default_value='20.0'
    )

    gap_follow = Node(
        package='pure_pursuit',
        executable='pure_pursuit_node_taylor',
        name='pure_pursuit',
        output='screen',
        parameters=[{'file': file}, {'la': lookahead}, {'speed': basespeed}, {'mxangle': mxangle}]
    )

    ld.add_action(file_launch_arg)
    ld.add_action(lookahead_launch_arg)
    ld.add_action(basespeed_launch_arg)
    ld.add_action(mxangle_launch_arg)
    ld.add_action(pure_pursuit)

    
    return ld
