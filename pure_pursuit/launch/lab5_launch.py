from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    file = LaunchConfiguration('file')
    mode = LaunchConfiguration('mode')
    lookahead = LaunchConfiguration('l')
    speed = LaunchConfiguration('speed')
    mxangle = LaunchConfiguration('mxangle')
    logging = LaunchConfiguration('logging')

    file_launch_arg = DeclareLaunchArgument(
        'file',
        default_value='/sim_ws/src/pure_pursuit/src/waypoints.csv'
    )

    mode_launch_arg = DeclareLaunchArgument(
        'mode',
        default_value='v'
    )

    logging_launch_arg = DeclareLaunchArgument(
        'logging',
        default_value='false'
    )
    
    lookahead_launch_arg = DeclareLaunchArgument(
        'l',
        default_value='1.0'
    )
    
    speed_launch_arg = DeclareLaunchArgument(
        'speed',
        default_value='1.0'
    )

    mxangle_launch_arg = DeclareLaunchArgument(
        'mxangle',
        default_value='20.0'
    )

    pure_pursuit = Node(
        package='pure_pursuit',
        executable='pure_pursuit_node',
        name='pure_pursuit',
        output='screen',
        parameters=[{'file': file}, {'l': lookahead}, {'mode': mode}, {'speed': speed}, {'mxangle': mxangle}, {'logging': logging}]
    )

    logger = Node(
        package='pure_pursuit',
        executable='logger_node',
        name='logger',
        output='screen',
        parameters=[{'file': file}, {'logging': logging}, {'mode': mode}]
    )

    ld.add_action(file_launch_arg)
    ld.add_action(lookahead_launch_arg)
    ld.add_action(speed_launch_arg)
    ld.add_action(mode_launch_arg)
    ld.add_action(mxangle_launch_arg)
    ld.add_action(logging_launch_arg)
    ld.add_action(pure_pursuit)
    ld.add_action(logger)

    
    return ld
