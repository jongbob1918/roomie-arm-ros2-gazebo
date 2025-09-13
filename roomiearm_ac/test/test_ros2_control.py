#!/usr/bin/env python3
"""
roomie_ac와 ros2_control 통합 테스트
"""
import asyncio
import rclpy
from rclpy.node import Node
import numpy as np

# roomie_ac 모듈들 import
from roomiearm_ac.robot_controller import RobotController
from roomiearm_ac.vision_controller import VisionController
from roomiearm_ac import config


class RoomieROS2ControlTest(Node):
    def __init__(self):
        super().__init__('roomie_ros2_control_test')
        
        # 시뮬레이션 모드로 로봇 컨트롤러 초기화
        self.robot_controller = RobotController(simulation_mode=True, node=self)
        
        # 비전 컨트롤러 초기화
        self.vision_controller = VisionController(self)
        
        self.get_logger().info("🚀 Roomie ros2_control 테스트 노드 시작!")
        
        # 테스트 시퀀스 타이머
        self.test_timer = self.create_timer(5.0, self.run_test_sequence)
        self.test_step = 0
        
    async def run_test_sequence(self):
        """단계별 테스트 시퀀스"""
        
        if self.test_step == 0:
            self.get_logger().info("=== 테스트 1: 홈 포지션으로 이동 ===")
            home_angles = np.array([0.0, 0.0, 0.0, 0.0])
            await self.robot_controller.move_to_angles_rad(home_angles)
            
        elif self.test_step == 1:
            self.get_logger().info("=== 테스트 2: 관절 각도 개별 테스트 ===")
            test_angles = np.array([0.5, -0.3, 0.8, -0.5])
            await self.robot_controller.move_to_angles_rad(test_angles)
            
        elif self.test_step == 2:
            self.get_logger().info("=== 테스트 3: IK를 이용한 3D 위치 이동 ===")
            target_xyz = np.array([0.2, 0.1, 0.3])  # 20cm 앞, 10cm 오른쪽, 30cm 위
            await self.robot_controller.move_to_pose_ik(target_xyz)
            
        elif self.test_step == 4:
            self.get_logger().info("=== 테스트 4: ArUco 마커 감지 시뮬레이션 ===")
            # 가상 ArUco 마커 위치 설정
            virtual_marker_pos = np.array([0.15, -0.05, 0.25])
            await self.robot_controller.move_to_pose_ik(virtual_marker_pos)
            
        elif self.test_step == 5:
            self.get_logger().info("=== 테스트 완료: 홈으로 복귀 ===")
            home_angles = np.array([0.0, 0.0, 0.0, 0.0])
            await self.robot_controller.move_to_angles_rad(home_angles)
            self.get_logger().info("✅ 모든 테스트 완료!")
            
        else:
            self.test_timer.cancel()
            return
        
        self.test_step += 1


def main(args=None):
    rclpy.init(args=args)
    
    test_node = RoomieROS2ControlTest()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        test_node.get_logger().info("테스트 중단됨")
    finally:
        test_node.robot_controller.disconnect()
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
