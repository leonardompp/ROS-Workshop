from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, LogInfo
from launch.event_handlers import OnExecutionComplete, OnProcessExit, OnProcessIO, OnProcessStart, OnShutdown
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    # Using event handlers throughout the file to ensure nodes execute in order
    # Out of order execution was causing race conditions in some machines
    ld = LaunchDescription()

    # File locations
    package_model_name = 'simulation_files'
    package_path = FindPackageShare(package_model_name)
    model_path = PathJoinSubstitution([package_path, 'urdf', 'robot.urdf'])

    # Start robot descriptor
    robot_description_content = ParameterValue(Command(['xacro ', model_path]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False
        }]
    )

    ld.add_action(robot_state_publisher_node)

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'use_sim_time': False
        }]
    )

    ld.add_action(joint_state_publisher_node)


    return ld