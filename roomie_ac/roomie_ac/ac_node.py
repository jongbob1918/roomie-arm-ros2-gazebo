# roomie_ac/ac_node.py - 단순화된 버전

# --- 기본 및 ROS 라이브러리 임포트 ---
import rclpy
import asyncio
import threading
import numpy as np
import traceback
import time
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# --- 사용자 정의 메시지 및 모듈 임포트 ---
from roomie_msgs.action import SetPose, ClickButton
from . import config
from .config_manager import ConfigManager
from .robot_controller import RobotController
from .vision_controller import VisionController
from .ros_joint_publisher import ROSJointPublisher


# ==============================================================================
# 🤖 메인 제어 노드 클래스 
# ==============================================================================
class ACNode(Node):
    def __init__(self, simulation_mode=False):
        """ROS2 메인 노드 - 단순화된 버전"""
        super().__init__('ac_node')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # --- 설정 관리자 초기화 (기존 config.py 호환성 유지) ---
        self.config_manager = ConfigManager(self, simulation_mode)
        self.config_manager.log_current_config()
        
        # 전역 config를 ConfigManager로 대체하여 기존 코드와 호환성 유지
        global config
        config = self.config_manager

        # --- 핵심 컨트롤러들 초기화 ---
        self.robot = RobotController(simulation_mode, node=self)
        self.vision = VisionController(self.robot, node=self)
        self.joint_publisher = ROSJointPublisher(callback_group=self.callback_group)
        
        # --- 중복 실행 방지를 위한 상태 변수 ---
        self._click_action_lock = threading.Lock()
        self._is_click_action_running = False
        self._last_action_info = {"button_id": None, "timestamp": 0.0}
        
        # --- ROS2 Action 서버 ---
        self.action_server = ActionServer(
            self,
            ClickButton,
            'click_button',
            self._click_button_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info("✅ ACNode 초기화 완료")
    
    async def _click_button_callback(self, goal_handle):
        """버튼 클릭 액션 콜백 - 단순한 조건문으로 처리"""
        button_id = goal_handle.request.button_id
        self.get_logger().info(f"🎯 버튼 클릭 요청: ID {button_id}")
        
        # 중복 실행 방지
        with self._click_action_lock:
            if self._is_click_action_running:
                self.get_logger().warn("이미 다른 버튼 클릭 작업이 진행 중입니다.")
                goal_handle.abort()
                return ClickButton.Result(success=False, message="중복 실행")
            
            self._is_click_action_running = True
        
        try:
            # 제어 전략에 따른 분기
            success = False
            if config.CONTROL_STRATEGY == config.ControlStrategy.MODEL_DIRECT_PRESS:
                success = await self._direct_press(button_id)
            elif config.CONTROL_STRATEGY == config.ControlStrategy.MODEL_STANDBY_PRESS:
                success = await self._standby_press(button_id)
            elif config.CONTROL_STRATEGY == config.ControlStrategy.PBVS_PRESS:
                success = await self._vision_guided_press(button_id, mode='iterative')
            elif config.CONTROL_STRATEGY == config.ControlStrategy.PBVS_ONESHOT_PRESS:
                success = await self._vision_guided_press(button_id, mode='oneshot')
            else:
                self.get_logger().error(f"지원하지 않는 제어 전략: {config.CONTROL_STRATEGY}")
                success = False
            
            # 결과 반환
            if success:
                self.get_logger().info(f"✅ 버튼 ID {button_id} 클릭 성공")
                goal_handle.succeed()
                return ClickButton.Result(success=True, message="성공")
            else:
                self.get_logger().error(f"❌ 버튼 ID {button_id} 클릭 실패")
                goal_handle.abort()
                return ClickButton.Result(success=False, message="실패")
                
        except Exception as e:
            error_msg = f"버튼 클릭 중 오류: {str(e)}"
            self.get_logger().error(error_msg)
            self.get_logger().error(traceback.format_exc())
            goal_handle.abort()
            return ClickButton.Result(success=False, message=error_msg)
            
        finally:
            # 상태 플래그 해제
            with self._click_action_lock:
                self._is_click_action_running = False
                self._last_action_info = {
                    "button_id": button_id,
                    "timestamp": time.time()
                }
    
    async def _direct_press(self, button_id: int) -> bool:
        """사전 정의된 좌표로 직접 이동"""
        self.get_logger().info(f">> 전략 1: 직접 이동 (버튼 ID {button_id})")
        
        # 사전 정의된 좌표 확인
        target_pose = config.PREDEFINED_BUTTON_POSES_M.get(button_id)
        if target_pose is None:
            self.get_logger().error(f"버튼 ID {button_id}의 좌표가 정의되지 않음")
            return False
        
        self.get_logger().info(f"목표 좌표: {np.round(target_pose, 3)}")
        
        # IK를 사용해 목표 위치로 이동
        if not await self.robot.move_to_pose_ik(target_pose, blocking=True):
            self.get_logger().error("목표 위치 이동 실패")
            return False
        
        await asyncio.sleep(1.0)  # 잠시 대기
        self.get_logger().info("직접 이동 완료")
        return True
    
    async def _standby_press(self, button_id: int) -> bool:
        """준비 위치를 거쳐 이동"""
        self.get_logger().info(f">> 전략 2: 준비 위치 경유 (버튼 ID {button_id})")
        
        target_pose = config.PREDEFINED_BUTTON_POSES_M.get(button_id)
        if target_pose is None:
            self.get_logger().error(f"버튼 ID {button_id}의 좌표가 정의되지 않음")
            return False
        
        # 준비 위치 계산 (목표에서 일정 거리 뒤)
        standby_pose = target_pose - np.array([config.SERVOING_STANDBY_DISTANCE_M, 0, 0])
        
        self.get_logger().info(f"준비 위치로 이동: {np.round(standby_pose, 3)}")
        if not await self.robot.move_to_pose_ik(standby_pose, blocking=True):
            self.get_logger().error("준비 위치 이동 실패")
            return False
        
        await asyncio.sleep(0.5)
        
        self.get_logger().info(f"목표 위치로 이동: {np.round(target_pose, 3)}")
        if not await self.robot.move_to_pose_ik(target_pose, blocking=True):
            self.get_logger().error("목표 위치 이동 실패")
            return False
        
        await asyncio.sleep(0.5)
        
        self.get_logger().info("준비 위치로 복귀")
        if not await self.robot.move_to_pose_ik(standby_pose, blocking=True):
            self.get_logger().error("준비 위치 복귀 실패")
            return False
        
        self.get_logger().info("준비 위치 경유 완료")
        return True
    
    async def _vision_guided_press(self, button_id: int, mode: str = 'oneshot') -> bool:
        """비전 가이드 이동 후 누르기"""
        self.get_logger().info(f">> 전략 3/4: 비전 가이드 ({mode}) (버튼 ID {button_id})")
        
        # 비전 서보잉으로 버튼 앞까지 이동
        if not await self.vision.visual_servo_to_button(button_id, mode):
            self.get_logger().error("비전 서보잉 실패")
            return False
        
        # 버튼 누르기
        if not await self.vision.press_button():
            self.get_logger().error("버튼 누르기 실패")
            return False
        
        self.get_logger().info("비전 가이드 버튼 누르기 완료")
        return True
    
    def destroy_node(self):
        """노드 종료 시 정리"""
        self.get_logger().info("ACNode 종료 중...")
        if hasattr(self, 'robot'):
            self.robot.disconnect()
        super().destroy_node()


# ==============================================================================
# 🚀 메인 함수
# ==============================================================================
def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)
    
    # 시뮬레이션 모드 파라미터 확인 (기본값: False)
    simulation_mode = False
    if args and '--simulation' in args:
        simulation_mode = True
        print("🎮 시뮬레이션 모드로 시작")
    else:
        print("🔧 실물 로봇 모드로 시작")
    
    try:
        # ACNode 생성
        node = ACNode(simulation_mode=simulation_mode)
        
        # MultiThreadedExecutor 사용
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        print("✅ ACNode 실행 중... (Ctrl+C로 종료)")
        executor.spin()
        
    except KeyboardInterrupt:
        print("🛑 사용자에 의한 종료")
    except Exception as e:
        print(f"❌ 오류 발생: {e}")
        print(traceback.format_exc())
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        print("👋 ACNode 종료 완료")


if __name__ == '__main__':
    main()
