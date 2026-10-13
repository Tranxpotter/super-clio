
import os
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.event_handlers import OnShutdown, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    this_pkg_name = 'bringup'
    this_pkg_dir = get_package_share_directory(this_pkg_name)
    super_odom_dir = get_package_share_directory("super_odometry")

    # Declare launch arguments
    flip = Node(
        package="bringup", 
        executable="livox_flip_node", 
        name="livox_flip_node", 
        output="screen", 
        parameters=[{
            "input_lidar_topic":"/livox/lidar", 
            "output_lidar_topic":"/livox/lidar_flipped", 
            "input_imu_topic": "/livox/imu", 
            "output_imu_topic": "/livox/imu_flipped"
        }]
    )

    super_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                super_odom_dir, 
                "launch", 
                "livox_mid360.launch.py"
            ])
        )
    )

    rviz = Node(
        package="rviz2", 
        executable="rviz2", 
        name="rviz2", 
        output="screen", 
        arguments=['-d', os.path.join(this_pkg_dir, "rviz", "default.rviz")]
    )



    return LaunchDescription([
        flip, 
        super_odom, 
        rviz, 
    ])