from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, LogInfo, DeclareLaunchArgument, ExecuteProcess
from launch.event_handlers import OnExecutionComplete, OnProcessExit, OnProcessIO, OnProcessStart, OnShutdown
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    # Start by generating the launch description
    ld = LaunchDescription()

    # Declare possible launch options which user can choose from
    sim_time = DeclareLaunchArgument(
      'use_sim_time', default_value='true'
    )
    ld.add_action(sim_time)

    # Find file locations
    # This will look in the install folder of the workspace, not in the package
    # Simualtion packages
    package_model_name = 'simulation_files'
    package_path = FindPackageShare(package_model_name)
    model_path = PathJoinSubstitution([package_path, 'urdf', 'robot.urdf'])
    # Gazebo IGN package
    ros_gz_sim_path = FindPackageShare('ros_gz_sim')

    # Start ignition
    # More info on this package at https://index.ros.org/p/ros_gz_sim/github-gazebosim-ros_gz/
    # Bear in mind that simulation related packages are not obvious to understand...
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_path, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf' # World to be spawned (and other options if wanted). Change for full path for different options
        }.items(), 
    )
    ld.add_action(gz_sim)

    # Bridge the relevant topics
    # For info on bridging, check https://gazebosim.org/docs/fortress/ros2_integration/ and https://index.ros.org/p/ros_gz_bridge/
    topic_names = ['/clock']
    ros_msg_types = ['rosgraph_msgs/msg/Clock']
    ign_msg_types = ['ignition.msgs.Clock']
    topic_directions = ['[']
    bridge_nodes = []

    for i in range(len(topic_names)):
        bridge_nodes.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name=topic_names[i][1:] + '_bridge',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }],
                arguments=[
                    topic_names[i] + '@' + ros_msg_types[i] + topic_directions[i] + ign_msg_types[i]
                ]
            )
        )
        ld.add_action(bridge_nodes[-1])


    # Commands can be run inside the launch file and be saves as ros parameters for nodes to access
    robot_description_content = ParameterValue(Command(['xacro ', model_path]), value_type=str)

    # Publish description of robot links
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    ld.add_action(robot_state_publisher_node)

    # Pubish the moving joints (wheels)
    """
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )    
    ld.add_action(joint_state_publisher_node)
    """

    # Spawn robot in Gazebo
    # This node quits after done
    # TODO: Change timeout condition
    spawn_urdf_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robot', '-topic', '/robot_description', '-z', '0.105'
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
    )
    ld.add_action(spawn_urdf_node)

    # After the robot has been spawned in Ignition, load controllers
    diff_drive_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments= [
            'diff_drive'
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    diff_drive_node_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_urdf_node,
            on_exit=[
                LogInfo(msg="Robot spawned - starting Diff Drive"),
                diff_drive_node
            ]
        )
    )
    ld.add_action(diff_drive_node_handler)

    joint_bradcaster_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments= [
            'joint_broadcaster'
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    joint_bradcaster_node_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_urdf_node,
            on_exit=[
                LogInfo(msg="Robot spawned - starting Joint Broadcast"),
                joint_bradcaster_node
            ]
        )
    )
    ld.add_action(joint_bradcaster_node_handler)


    return ld