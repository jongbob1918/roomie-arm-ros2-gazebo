#!/usr/bin/env python3

"""
Roomie 4DOF Robot 빈 월드 시뮬레이션 (ros2_control 기반)
빈 월드에서 로봇만으로 ros2_control 테스트용
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument("gui", default_value="true", description="Start Gazebo GUI")
    )
    declared_arguments.append(
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time")
    )
    declared_arguments.append(
        DeclareLaunchArgument("spawn_x", default_value="0.0", description="Robot spawn X position")
    )
    declared_arguments.append(
        DeclareLaunchArgument("spawn_y", default_value="0.0", description="Robot spawn Y position")
    )
    declared_arguments.append(
        DeclareLaunchArgument("spawn_z", default_value="0.0", description="Robot spawn Z position")
    )

    # Empty world gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", " -r -v 3 empty.sdf")],
        condition=IfCondition(gui),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", ["--headless-rendering -s -r -v 3 empty.sdf"])],
        condition=UnlessCondition(gui),
    )

    # Gazebo bridge (시계만 - 카메라 없음)
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("roomiearm_description"), "urdf/robots", "roomiearm.urdf.xacro"]
            ),
            " ",
            "use_gazebo:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("roomiearm_gazebo"),
            "config",
            "ros2_controllers.yaml",
        ]
    )

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time}
        ],
    )

    # Robot Spawner
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", "roomiearm",
            "-allow_renaming", "true",
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", spawn_z,
        ],
    )

    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Forward Position Controller Spawner
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--param-file", robot_controllers],
        output="screen",
    )

    nodes = [
        gazebo,
        gazebo_headless,
        gazebo_bridge,
        node_robot_state_publisher,
        gz_spawn_entity,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
