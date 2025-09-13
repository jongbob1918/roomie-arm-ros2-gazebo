#!/usr/bin/env python3

"""
roomiearm_4dof 로봇 state publisher 및 RViz 런치 파일
roomiearm_description의 URDF/xacro를 사용한 로봇 상태 퍼블리싱
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Launch description 생성"""

    # --- 1. 주요 패키지 및 파일 경로 설정 ---
    pkg_share = FindPackageShare(package='roomiearm_description').find('roomiearm_description')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz') # RViz 설정 파일 경로
    default_urdf_model_path = os.path.join(pkg_share, 'urdf/robots/roomiearm.urdf.xacro') # URDF 파일 경로


    # --- 2. 런치 인자(Launch Arguments) 선언 ---
    # 사용자가 launch 시점에 값을 바꿀 수 있는 변수들
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')
    urdf_model = LaunchConfiguration('urdf_model')
    use_gazebo = LaunchConfiguration('use_gazebo') # URDF로 전달할 인자

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Flag to enable joint_state_publisher_gui'
    )
    declare_rviz_arg = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RVIZ'
    )
    declare_urdf_model_arg = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot urdf file'
    )
    declare_use_gazebo_arg = DeclareLaunchArgument(
        name='use_gazebo',
        default_value='false',
        description='Whether to use Gazebo simulation specific tags'
    )

    # --- 3. URDF 파일을 읽고 robot_description 파라미터 생성 ---
    # xacro 명령어를 실행할 때 use_gazebo 인자를 전달
    robot_description_content = Command([
        'xacro ', urdf_model, ' ',
        'use_gazebo:=', use_gazebo
    ])

    # --- 4. 노드(Node) 정의 ---
    # Robot State Publisher 노드
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description_content,
                'use_sim_time': use_sim_time
            }
        ]
    )

    # Joint State Publisher GUI 노드
    # 'gui' 인자가 'true'일 때만 실행 (IfCondition)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(gui)
    )

    # RViz2 노드
    # 'use_rviz' 인자가 'true'일 때만 실행
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path],
        condition=IfCondition(use_rviz)
    )

    # --- 5. 런치 파일에 모든 구성 요소 추가 ---
    return LaunchDescription([
        # 인자 선언
        declare_use_sim_time_arg,
        declare_gui_arg,
        declare_rviz_arg,
        declare_urdf_model_arg,
        declare_use_gazebo_arg,

        # 노드 실행
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])