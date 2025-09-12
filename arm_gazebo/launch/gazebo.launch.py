#!/usr/bin/env python3

"""
Roomie 4DOF Robot Gazebo 시뮬레이션 런치 파일 (ros2_control 기반)
ArUco 마커가 있는 환경에서 ros2_control을 사용한 로봇 제어
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch description 생성"""
    
    # 패키지 경로
    pkg_arm_gazebo = FindPackageShare('arm_gazebo')
    pkg_arm_description = FindPackageShare('arm_description')
    
    # Gazebo 모델 경로 설정 (ArUco 마커 모델 포함)
    models_path = PathJoinSubstitution([pkg_arm_gazebo, 'models'])
    current_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '/opt/ros/jazzy/share')
    
    # 환경변수 설정
    set_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models_path, ':', current_gz_path]
    )
    
    # Launch arguments
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    
    # Arguments 선언
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument('gui', default_value='true', description='Launch Gazebo GUI')
    )
    declared_arguments.append(
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time')
    )
    declared_arguments.append(
        DeclareLaunchArgument('spawn_x', default_value='0.0', description='Robot spawn X position')
    )
    declared_arguments.append(
        DeclareLaunchArgument('spawn_y', default_value='0.0', description='Robot spawn Y position')
    )
    declared_arguments.append(
        DeclareLaunchArgument('spawn_z', default_value='0.0', description='Robot spawn Z position')
    )
    
    # World 파일 경로
    world_path = PathJoinSubstitution([
        pkg_arm_gazebo, 'worlds', 'roomie_aruco_world.world'
    ])
    
    # Gazebo 실행 (GUI 있는 버전)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"
        ]),
        launch_arguments=[("gz_args", [" -r -v 3 ", world_path])],
        condition=IfCondition(gui),
    )
    
    # Gazebo 실행 (Headless 버전)
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"
        ]),
        launch_arguments=[("gz_args", ["--headless-rendering -s -r -v 3 ", world_path])],
        condition=UnlessCondition(gui),
    )
    
    # ROS-Gazebo Bridge (카메라와 클록)
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    
    # URDF 생성 (ros2_control 사용)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([pkg_arm_description, "urdf", "roomie_4dof.xacro"]),
        " ",
        "use_gazebo:=true"
    ])
    robot_description = {"robot_description": robot_description_content}
    
    # 컨트롤러 설정 파일
    robot_controllers = PathJoinSubstitution([
        pkg_arm_gazebo, "config", "roomie_controllers.yaml"
    ])
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time}
        ],
    )
    
    # Robot Spawner
    robot_spawner = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", "roomie_4dof",
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
        parameters=[{"use_sim_time": use_sim_time}],
    )
    
    # Forward Position Controller Spawner
    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--param-file", robot_controllers],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    
    nodes = [
        set_model_path,
        gazebo,
        gazebo_headless,
        gazebo_bridge,
        robot_state_publisher,
        robot_spawner,
        joint_state_broadcaster_spawner,
        forward_position_controller_spawner,
    ]
    
    return LaunchDescription(declared_arguments + nodes)
