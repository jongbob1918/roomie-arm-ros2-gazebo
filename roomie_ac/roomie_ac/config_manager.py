"""
Config Manager - 기존 config.py와 호환성 유지하면서 시뮬레이션/실물 모드 지원
"""

import rclpy
from rclpy.node import Node
from . import config  # 기존 config.py 그대로 사용


class ConfigManager:
    """
    기존 config.py의 모든 변수를 유지하면서 
    시뮬레이션/실물 모드에 따른 차이점만 관리
    """
    
    def __init__(self, node: Node, simulation_mode: bool = False):
        self.node = node
        self.simulation_mode = simulation_mode
        
        # 기존 config.py의 모든 속성을 그대로 복사
        for attr in dir(config):
            if not attr.startswith('_'):
                setattr(self, attr, getattr(config, attr))
        
        # 시뮬레이션/실물 모드에 따른 차이점만 오버라이드
        self._apply_mode_specific_config()
    
    def _apply_mode_specific_config(self):
        """시뮬레이션/실물 모드별 설정 차이점 적용"""
        
        if self.simulation_mode:
            # 시뮬레이션 모드 설정
            self.CAMERA_TOPIC = "/camera_head/color/image_raw"
            self.JOINT_COMMAND_TOPIC = "/joint_group_position_controller/commands"
            self.JOINT_STATE_TOPIC = "/joint_states"
            self.USE_SIM_TIME = True
            
            # 시뮬레이션에서는 시리얼 포트 사용 안함
            self.USE_SERIAL = False
            
            # 시뮬레이션용 버튼 위치 (Gazebo world에 맞게 조정)
            self.PREDEFINED_BUTTON_POSES_M.update({
                101: [0.23, 0.0, 0.32],   # Gazebo world의 ArUco 마커 위치
                102: [0.23, 0.0, 0.16],  # Gazebo world의 ArUco 마커 위치
            })
            
        else:
            # 실물 로봇 모드 설정 (기존 config.py 값 유지)
            self.CAMERA_TOPIC = "/camera/image_raw"
            self.USE_SIM_TIME = False
            self.USE_SERIAL = True
            
            # 실물 로봇의 기존 설정들은 그대로 유지
            # (SERIAL_PORT, SERIAL_BAUD_RATE, PREDEFINED_BUTTON_POSES_M 등)
    
    def get_camera_topic(self) -> str:
        """카메라 토픽 이름 반환"""
        return self.CAMERA_TOPIC
    
    def should_use_serial(self) -> bool:
        """시리얼 통신 사용 여부"""
        return self.USE_SERIAL
    
    def is_simulation_mode(self) -> bool:
        """시뮬레이션 모드 여부"""
        return self.simulation_mode
    
    def log_current_config(self):
        """현재 설정 로깅"""
        mode = "SIMULATION" if self.simulation_mode else "REAL ROBOT"
        self.node.get_logger().info(f"Config loaded for {mode} mode")
        self.node.get_logger().info(f"Camera topic: {self.CAMERA_TOPIC}")
        self.node.get_logger().info(f"Control strategy: {self.CONTROL_STRATEGY}")
        if hasattr(self, 'USE_SERIAL'):
            self.node.get_logger().info(f"Use serial: {self.USE_SERIAL}")
