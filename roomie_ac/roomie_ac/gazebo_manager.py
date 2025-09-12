#!/usr/bin/env python3

"""
Gazebo Manager for roomie_ac
ROS2-Gazebo 통신을 위한 하드웨어 추상화 계층
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np
from typing import List, Dict, Optional
import time


class GazeboManager:
    """
    Gazebo 시뮬레이션과의 통신을 관리하는 클래스
    roomie_ac의 motion_controller.py에서 사용됨
    """
    
    def __init__(self, node: Node):
        self.node = node
        self.logger = node.get_logger()
        
        # Joint command publishers (ROS → Gazebo)
        self.joint_publishers = {}
        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        
        for joint_name in joint_names:
            topic_name = f'/{joint_name}_cmd'
            self.joint_publishers[joint_name] = self.node.create_publisher(
                Float64, 
                topic_name, 
                10
            )
            self.logger.info(f'Created publisher for {topic_name}')
        
        # Joint state subscriber (Gazebo → ROS)
        self.joint_state_sub = self.node.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Current joint states
        self.current_joint_states = {}
        self.joint_state_received = False
        
        self.logger.info('GazeboManager initialized successfully')
    
    def joint_state_callback(self, msg: JointState):
        """Joint state 콜백 - Gazebo에서 현재 관절 상태 수신"""
        for i, name in enumerate(msg.name):
            if name in ['joint_1', 'joint_2', 'joint_3', 'joint_4']:
                self.current_joint_states[name] = {
                    'position': msg.position[i],
                    'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                    'effort': msg.effort[i] if i < len(msg.effort) else 0.0
                }
        self.joint_state_received = True
    
    def send_joint_command(self, joint_angles: List[float]) -> bool:
        """
        관절 각도 명령을 Gazebo로 전송
        
        Args:
            joint_angles: [joint_1, joint_2, joint_3, joint_4] 각도 (라디안)
            
        Returns:
            bool: 전송 성공 여부
        """
        try:
            if len(joint_angles) != 4:
                self.logger.error(f'Expected 4 joint angles, got {len(joint_angles)}')
                return False
            
            joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
            
            for i, (joint_name, angle) in enumerate(zip(joint_names, joint_angles)):
                cmd_msg = Float64()
                cmd_msg.data = float(angle)
                self.joint_publishers[joint_name].publish(cmd_msg)
                
            self.logger.debug(f'Sent joint commands: {joint_angles}')
            return True
            
        except Exception as e:
            self.logger.error(f'Failed to send joint command: {e}')
            return False
    
    def get_current_joint_angles(self) -> Optional[List[float]]:
        """
        현재 관절 각도를 반환
        
        Returns:
            List[float]: [joint_1, joint_2, joint_3, joint_4] 각도 (라디안)
            None: 아직 데이터를 받지 못한 경우
        """
        if not self.joint_state_received:
            return None
        
        try:
            joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
            angles = []
            
            for joint_name in joint_names:
                if joint_name in self.current_joint_states:
                    angles.append(self.current_joint_states[joint_name]['position'])
                else:
                    self.logger.warn(f'No data for {joint_name}')
                    return None
            
            return angles
            
        except Exception as e:
            self.logger.error(f'Failed to get current joint angles: {e}')
            return None
    
    def wait_for_connection(self, timeout_sec: float = 5.0) -> bool:
        """
        Gazebo와의 연결을 기다림
        
        Args:
            timeout_sec: 타임아웃 시간 (초)
            
        Returns:
            bool: 연결 성공 여부
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout_sec:
            if self.joint_state_received:
                # Publisher가 연결되었는지 확인
                all_connected = True
                for joint_name, pub in self.joint_publishers.items():
                    if pub.get_subscription_count() == 0:
                        all_connected = False
                        break
                
                if all_connected:
                    self.logger.info('Successfully connected to Gazebo')
                    return True
            
            time.sleep(0.1)
        
        self.logger.error(f'Failed to connect to Gazebo within {timeout_sec} seconds')
        return False
    
    def is_connected(self) -> bool:
        """
        Gazebo와 연결되어 있는지 확인
        
        Returns:
            bool: 연결 상태
        """
        if not self.joint_state_received:
            return False
        
        # 최소 하나의 publisher가 연결되어 있는지 확인
        for pub in self.joint_publishers.values():
            if pub.get_subscription_count() > 0:
                return True
        
        return False
    
    def get_joint_limits(self) -> Dict[str, Dict[str, float]]:
        """
        관절 제한 값 반환 (URDF에서 정의된 값)
        
        Returns:
            Dict: 각 관절의 위치 제한값
        """
        return {
            'joint_1': {'min': -1.54, 'max': 1.54},  # ±88.3도
            'joint_2': {'min': -1.54, 'max': 1.54},
            'joint_3': {'min': -1.54, 'max': 1.54},
            'joint_4': {'min': -1.54, 'max': 1.54}
        }
    
    def check_joint_limits(self, joint_angles: List[float]) -> bool:
        """
        관절 각도가 제한 범위 내에 있는지 확인
        
        Args:
            joint_angles: 확인할 관절 각도들
            
        Returns:
            bool: 모든 관절이 제한 범위 내에 있으면 True
        """
        limits = self.get_joint_limits()
        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        
        for i, (joint_name, angle) in enumerate(zip(joint_names, joint_angles)):
            if joint_name in limits:
                if not (limits[joint_name]['min'] <= angle <= limits[joint_name]['max']):
                    self.logger.error(
                        f'{joint_name} angle {angle:.3f} exceeds limits '
                        f'[{limits[joint_name]["min"]:.3f}, {limits[joint_name]["max"]:.3f}]'
                    )
                    return False
        
        return True
