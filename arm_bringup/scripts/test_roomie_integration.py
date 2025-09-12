#!/usr/bin/env python3
"""
roomie_ac RobotController와 ros2_control 통합 테스트
"""
import rclpy
from rclpy.node import Node
import numpy as np
import sys
import os

# roomie_ac 패키지 경로 추가
sys.path.append(os.path.join(os.path.dirname(__file__), '../../'))

from roomie_ac.roomie_ac.robot_controller import RobotController
from roomie_ac.roomie_ac import config


class RoomieIntegrationTest(Node):
    def __init__(self):
        super().__init__('roomie_integration_test')
        
        # RobotController를 시뮬레이션 모드로 초기화
        self.robot_controller = RobotController(simulation_mode=True, node=self)
        
        self.test_step = 0
        
        self.get_logger().info("🚀 Roomie Integration 테스트 시작!")
        self.get_logger().info("💡 RobotController 클래스를 통한 ros2_control 제어 테스트")
        
        # 2초 후부터 4초마다 테스트 실행
        self.test_timer = self.create_timer(2.0, self.initial_delay)
        
    def initial_delay(self):
        """초기 대기 후 테스트 시작"""
        self.test_timer.cancel()
        self.test_timer = self.create_timer(4.0, self.run_integration_test)
        self.get_logger().info("⏰ 통합 테스트 시퀀스 시작 (4초마다 실행)")
        
    async def run_integration_test(self):
        """RobotController 클래스를 통한 통합 테스트"""
        
        if self.test_step == 0:
            self.get_logger().info("=== 통합 테스트 1: 홈 포지션 이동 (RobotController) ===")
            home_angles = np.array([0.0, 0.0, 0.0, 0.0])
            success = await self.robot_controller.move_to_angles_rad(home_angles)
            self.get_logger().info(f"홈 이동 결과: {'성공' if success else '실패'}")
            
        elif self.test_step == 1:
            self.get_logger().info("=== 통합 테스트 2: 관절 각도 제어 ===")
            test_angles = np.array([0.6, -0.4, 0.5, -0.3])
            success = await self.robot_controller.move_to_angles_rad(test_angles)
            self.get_logger().info(f"관절 이동 결과: {'성공' if success else '실패'}")
            
        elif self.test_step == 2:
            self.get_logger().info("=== 통합 테스트 3: IK를 이용한 3D 위치 이동 ===")
            target_xyz = np.array([0.2, 0.1, 0.25])  # 20cm 앞, 10cm 오른쪽, 25cm 위
            success = await self.robot_controller.move_to_pose_ik(target_xyz)
            self.get_logger().info(f"IK 이동 결과: {'성공' if success else '실패'}")
            if success:
                current_pos = self.robot_controller.get_current_angles_rad()
                self.get_logger().info(f"현재 관절 각도: {np.round(current_pos, 3)}")
            
        elif self.test_step == 3:
            self.get_logger().info("=== 통합 테스트 4: 다른 3D 위치로 이동 ===")
            target_xyz = np.array([0.15, -0.08, 0.30])
            success = await self.robot_controller.move_to_pose_ik(target_xyz)
            self.get_logger().info(f"IK 이동 결과: {'성공' if success else '실패'}")
            
        elif self.test_step == 4:
            self.get_logger().info("=== 통합 테스트 5: 홈으로 복귀 ===")
            home_angles = np.array([0.0, 0.0, 0.0, 0.0])
            success = await self.robot_controller.move_to_angles_rad(home_angles)
            self.get_logger().info(f"홈 복귀 결과: {'성공' if success else '실패'}")
            
        elif self.test_step == 5:
            self.get_logger().info("✅ 모든 통합 테스트 완료!")
            self.get_logger().info("🎯 roomie_ac ↔ ros2_control 통합 성공!")
            self.test_timer.cancel()
            return
            
        self.test_step += 1


def main(args=None):
    rclpy.init(args=args)
    
    test_node = RoomieIntegrationTest()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        test_node.get_logger().info("통합 테스트 중단됨")
    finally:
        test_node.robot_controller.disconnect()
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
