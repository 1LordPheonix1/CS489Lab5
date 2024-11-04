from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # kp = LaunchConfiguration('kp')
    # ki = LaunchConfiguration('ki')
    # kd = LaunchConfiguration('kd')
    # theta = LaunchConfiguration('theta')
    # l = LaunchConfiguration('l')
    # dist = LaunchConfiguration('dist')

    # kp_launch_arg = DeclareLaunchArgument(
    #     'kp',
    #     default_value='2.0'
    # )

    # ki_launch_arg = DeclareLaunchArgument(
    #     'ki',
    #     default_value='0.005'
    # )

    # kd_launch_arg = DeclareLaunchArgument(
    #     'kd',
    #     default_value='0.001'
    # )

    # theta_launch_arg = DeclareLaunchArgument(
    #     'theta',
    #     default_value='60.0'
    # )

    # l_launch_arg = DeclareLaunchArgument(
    #     'l',
    #     default_value='1.0'
    # )

    # dist_launch_arg = DeclareLaunchArgument(
    #     'dist',
    #     default_value='0.9'
    # )


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
