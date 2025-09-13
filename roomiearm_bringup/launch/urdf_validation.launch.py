#!/usr/bin/env python3
"""
URDF 검증 모드 런치 파일
- Gazebo 없이 URDF 구조만 검증
- Joint State Publisher GUI로 빠른 조인트 테스트
- 개발 단계에서 URDF 수정 시 사용
"""

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """URDF 검증 모드 런치 생성"""
    
    # 패키지 경로
    pkg_arm_description = FindPackageShare('roomiearm_description')
    
    # URDF 생성 (Gazebo 사용 안함)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([pkg_arm_description, "urdf", "roomiearm.urdf.xacro"]),
        " ",
        "use_gazebo:=false"  # Gazebo 플러그인 비활성화
    ])
    robot_description = {"robot_description": robot_description_content}
    
    # Robot State Publisher (tf 발행)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": False}  # 실제 시간 사용
        ],
    )
    
    # Joint State Publisher GUI (조인트 제어)
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )
    
    # RViz (선택사항 - 자동 실행하지 않음)
    # 사용자가 원할 때 별도로 실행: rviz2 -d $(ros2 pkg prefix arm_description)/share/arm_description/rviz/urdf_config.rviz
    
    nodes = [
        robot_state_publisher,
        joint_state_publisher_gui,
    ]
    
    return LaunchDescription(nodes)
