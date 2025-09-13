#!/usr/bin/env python3
"""
roomie_ac와 ros2_control 통합 테스트 노드
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import threading
import time


class RoomieROS2ControlTest(Node):
    def __init__(self):
        super().__init__('roomie_ros2_control_test')
        
        # ros2_control 명령 퍼블리셔
        self.position_command_pub = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            10
        )
        
        # joint_states 구독
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_joint_positions = np.zeros(4)
        self.test_step = 0
        
        self.get_logger().info("🚀 Roomie ros2_control 테스트 노드 시작!")
        
        # 1초 후부터 5초마다 테스트 실행
        self.test_timer = self.create_timer(1.0, self.initial_delay)
        
    def initial_delay(self):
        """초기 1초 대기 후 테스트 시작"""
        self.test_timer.cancel()
        self.test_timer = self.create_timer(3.0, self.run_test_sequence)
        self.get_logger().info("⏰ 테스트 시퀀스 시작 (3초마다 실행)")
        
    def joint_state_callback(self, msg):
        """조인트 상태 콜백"""
        try:
            # joint_1, joint_2, joint_3, joint_4 순서로 저장
            joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
            positions = []
            
            for joint_name in joint_names:
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    positions.append(msg.position[idx])
            
            if len(positions) == 4:
                self.current_joint_positions = np.array(positions)
                
        except Exception as e:
            self.get_logger().error(f"조인트 상태 콜백 오류: {e}")
    
    def send_joint_command(self, angles_rad):
        """조인트 각도 명령 전송"""
        command_msg = Float64MultiArray()
        command_msg.data = angles_rad.tolist()
        
        self.position_command_pub.publish(command_msg)
        
        self.get_logger().info(f"명령 전송: {np.round(angles_rad, 3)} rad")
        self.get_logger().info(f"현재 위치: {np.round(self.current_joint_positions, 3)} rad")
        
    def run_test_sequence(self):
        """단계별 테스트 시퀀스"""
        
        if self.test_step == 0:
            self.get_logger().info("=== 테스트 1: 홈 포지션으로 이동 ===")
            home_angles = np.array([0.0, 0.0, 0.0, 0.0])
            self.send_joint_command(home_angles)
            
        elif self.test_step == 1:
            self.get_logger().info("=== 테스트 2: Joint 1 회전 테스트 ===")
            test_angles = np.array([0.8, 0.0, 0.0, 0.0])
            self.send_joint_command(test_angles)
            
        elif self.test_step == 2:
            self.get_logger().info("=== 테스트 3: Joint 2 회전 테스트 ===")
            test_angles = np.array([0.8, -0.5, 0.0, 0.0])
            self.send_joint_command(test_angles)
            
        elif self.test_step == 3:
            self.get_logger().info("=== 테스트 4: Joint 3 회전 테스트 ===")
            test_angles = np.array([0.8, -0.5, 0.7, 0.0])
            self.send_joint_command(test_angles)
            
        elif self.test_step == 4:
            self.get_logger().info("=== 테스트 5: Joint 4 회전 테스트 ===")
            test_angles = np.array([0.8, -0.5, 0.7, -0.6])
            self.send_joint_command(test_angles)
            
        elif self.test_step == 5:
            self.get_logger().info("=== 테스트 6: 복합 동작 테스트 ===")
            test_angles = np.array([0.3, 0.4, -0.2, 0.5])
            self.send_joint_command(test_angles)
            
        elif self.test_step == 6:
            self.get_logger().info("=== 테스트 7: 홈으로 복귀 ===")
            home_angles = np.array([0.0, 0.0, 0.0, 0.0])
            self.send_joint_command(home_angles)
            
        elif self.test_step == 7:
            self.get_logger().info("✅ 모든 테스트 완료!")
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
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
