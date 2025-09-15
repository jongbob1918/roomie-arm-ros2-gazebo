#!/usr/bin/env python3
"""
RoomieArm Core 통합 테스트 런치 파일
- Gazebo 시뮬레이션 + roomiearm_core 노드들 + GUI 테스트 클라이언트
- 모든 것을 한 번에 실행하여 완전한 테스트 환경 제공
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """RoomieArm Core 통합 테스트 런치 생성"""

    # Launch arguments
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')

    # Arguments 선언
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument('gui', default_value='true', description='Launch Gazebo GUI')
    )
    declared_arguments.append(
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time')
    )
    declared_arguments.append(
        DeclareLaunchArgument('log_level', default_value='INFO', description='Log level (DEBUG, INFO, WARN, ERROR)')
    )

    # 1. 기본 시뮬레이션 실행 (Gazebo + 로봇 + ros2_control)
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("roomiearm_bringup"), "/launch/simulation_control.launch.py"
        ]),
        launch_arguments=[
            ("gui", gui),
            ("use_sim_time", use_sim_time)
        ]
    )

    # 2. RoomieArm Core - Vision Node (ArUco 마커 감지)
    vision_node = Node(
        package="roomiearm_core",
        executable="vision_node",
        name="vision_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # 3. RoomieArm Core - Button Click Node (액션 서버)
    button_click_node = Node(
        package="roomiearm_core",
        executable="button_click_node",
        name="button_click_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # 4. GUI 테스트 클라이언트 (설치된 실행파일 사용)
    gui_test_client = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([
                FindPackageShare("roomiearm_bringup"),
                "lib", "roomiearm_bringup", "gui_test_client.py"
            ])
        ],
        name="gui_test_client",
        output="screen",
    )

    # 지연된 노드 실행 (Gazebo 완전 로딩 대기)
    delayed_vision_node = TimerAction(
        period=5.0,  # 5초 후 실행
        actions=[vision_node]
    )

    delayed_button_click_node = TimerAction(
        period=7.0,  # 7초 후 실행
        actions=[button_click_node]
    )

    # GUI 클라이언트는 수동 실행 (PyQt6 의존성 문제로 인해 비활성화)
    # delayed_gui_client = TimerAction(
    #     period=10.0,  # 10초 후 실행
    #     actions=[gui_test_client]
    # )

    nodes = [
        simulation_launch,
        delayed_vision_node,
        delayed_button_click_node,
        # delayed_gui_client,  # 수동 실행으로 변경
    ]

    return LaunchDescription(declared_arguments + nodes)