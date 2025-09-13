# roomie_arm_control/ros_joint_publisher.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np 
from . import config # config.py 임포트
from rclpy.callback_groups import ReentrantCallbackGroup

class ROSJointPublisher(Node):
    def __init__(self, callback_group: ReentrantCallbackGroup):
        super().__init__('joint_publisher')
        
        # ✨ create_publisher에 콜백 그룹을 전달
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10, callback_group=callback_group)
        self.get_logger().info('ROSJointPublisher 노드 초기화됨.')

    def publish(self, joint_names, joint_positions_rad):
        """
        로봇의 현재 관절 상태를 /joint_states 토픽으로 발행합니다.
        joint_names: 관절 이름 리스트 (예: ['joint_1', 'joint_2', ...])
        joint_positions_rad: 각 관절의 현재 위치 (라디안, numpy 배열)
        """
        msg = JointState()
        # 메시지 헤더의 타임스탬프를 현재 시간으로 설정합니다.
        msg.header.stamp = self.get_clock().now().to_msg()
        # 관절 이름을 설정합니다.
        msg.name = joint_names
        # 관절 위치를 설정합니다. (라디안 단위, 파이썬 리스트로 변환)
        msg.position = joint_positions_rad.tolist()

        # 메시지를 발행합니다.
        self.publisher_.publish(msg)

        if config.DEBUG: # config.DEBUG 사용
            self.get_logger().info(f"  [ROS2 TX] -> /joint_states Published: {np.round(joint_positions_rad, 4)}")