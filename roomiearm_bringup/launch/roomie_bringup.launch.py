#!/usr/bin/env python3

"""
Roomie 4DOF 로봇 통합 런치 파일
roomiearm_description + roomiearm_ac 통합 실행
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch description 생성"""
    
    # 패키지 경로
    pkg_roomiearm_description = FindPackageShare('roomiearm_description')
    pkg_roomiearm_ac = FindPackageShare('roomiearm_ac')
    
    # Launch arguments
    simulation_mode = LaunchConfiguration('simulation_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    
    declare_simulation_mode = DeclareLaunchArgument(
        'simulation_mode',
        default_value='false',
        description='Use simulation mode if true (Gazebo), real robot if false'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Show joint_state_publisher_gui if true'
    )
    
    # Robot State Publisher 런치 (arm_description에서)
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_roomiearm_description,
            '/launch/robot_state_publisher.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Roomie AC Node (roomie_ac 패키지의 메인 제어 노드)
    roomie_ac_node = Node(
        package='roomiearm_ac',
        executable='ac_node',
        name='arm_control_node',
        output='screen',
        parameters=[
            {
                'simulation_mode': simulation_mode,
                'use_sim_time': use_sim_time
            }
        ],
        remappings=[
            # Gazebo 토픽을 arm_description 표준에 맞춤
            ('/arm_controller/joint_trajectory', '/joint_trajectory_controller/joint_trajectory'),
            ('/joint_states', '/joint_states'),
        ]
    )
    
    # RViz2 (시각화용 - 옵션)
    rviz_config_file = os.path.join(
        get_package_share_directory('roomiearm_description'),
        'rviz',
        'urdf_config.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(gui)
    )
    
    return LaunchDescription([
        declare_simulation_mode,
        declare_use_sim_time,
        declare_gui,
        
        # Robot State Publisher
        robot_state_publisher_launch,
        
        # Main Control Node
        roomie_ac_node,
        
        # Visualization (optional)
        rviz_node,
    ])
