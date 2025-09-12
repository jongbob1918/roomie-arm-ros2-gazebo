#!/usr/bin/env python3

"""
roomie_4dof 로봇 state publisher 런치 파일
arm_description의 URDF/xacro를 사용한 로봇 상태 퍼블리싱
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch description 생성"""
    
    # 패키지 경로
    pkg_arm_description = FindPackageShare('arm_description')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    # URDF 파일 경로
    urdf_file = os.path.join(
        get_package_share_directory('arm_description'),
        'urdf',
        'roomie_4dof.xacro'
    )
    
    # robot_description 파라미터 생성
    robot_description = Command(['xacro ', urdf_file])
    
    # Robot State Publisher 노드
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }
        ]
    )
    
    # Joint State Publisher (수동 테스트용)
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher,
        joint_state_publisher
    ])
