#!/usr/bin/env python3
"""
시뮬레이션 제어 모드 런치 파일
- Gazebo + ros2_control 기반 실제 로봇 시뮬레이션
- RQT Joint Trajectory Controller로 전문적인 제어
- 실제 배포 환경과 동일한 구조
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """시뮬레이션 제어 모드 런치 생성"""
    
    # Launch arguments
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Arguments 선언
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument('gui', default_value='true', description='Launch Gazebo GUI')
    )
    declared_arguments.append(
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time')
    )
    
    # Gazebo 시뮬레이션 실행
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("roomiearm_gazebo"), "/launch/gazebo.launch.py"
        ]),
        launch_arguments=[
            ("gui", gui),
            ("use_sim_time", use_sim_time)
        ]
    )
    
    # Robot GUI Controller (슬라이더 제어)
    robot_gui_controller = Node(
        package="roomiearm_bringup",
        executable="robot_gui_controller.py",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    
    # RViz2 (로봇 시각화)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("roomiearm_description"), 
        "rviz", 
        "urdf_config.rviz"
    ])
    
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    
    nodes = [
        gazebo_launch,
        robot_gui_controller,
        rviz2,
    ]
    
    return LaunchDescription(declared_arguments + nodes)
