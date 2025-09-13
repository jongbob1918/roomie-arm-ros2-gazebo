import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # URDF 파일 경로 및 내용 로드
    urdf_file_path = os.path.join(
        get_package_share_directory('roomiearm_ac'), 'urdf', 'roomie2.urdf')
    with open(urdf_file_path, 'r') as f:
        robot_description_content = f.read()

    # 1. robot_state_publisher 노드 (RViz 시각화를 위해 필수)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # 2. ac_node (메인 제어 노드)
    ac_node = Node(
        package='roomiearm_ac',
        executable='ac_node',
        name='ac_node',
        output='screen'
    )

    # 3. RViz2 노드
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
    )

    # 4. 위에서 정의한 필수 노드들만 LaunchDescription에 담아 반환
    return LaunchDescription([
        robot_state_publisher_node,
        ac_node,
        rviz2_node
    ])