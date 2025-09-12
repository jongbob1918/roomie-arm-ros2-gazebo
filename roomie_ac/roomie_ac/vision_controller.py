"""
비전 기능 통합 클래스 - ArUco 감지, 좌표 변환, 이미지 서보잉
"""
import asyncio
import os
import numpy as np
import cv2
import cv2.aruco as aruco
from typing import Optional, Tuple, Dict
from geometry_msgs.msg import Point, Quaternion, Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from . import config
# TODO: VisionClient와 CoordinateTransformer 통합 필요
# from .vision_client import VisionServiceClient
# from .coordinate_transformer import CoordinateTransformer
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class VisionController:
    """ArUco 감지, 좌표 변환, 이미지 서보잉 통합"""
    
    def __init__(self, node: Optional[Node] = None, 
                 config_manager=None,
                 robot_controller=None):
        """
        Args:
            node: ROS2 노드 (이미지 구독용)
            config_manager: 설정 관리자
            robot_controller: 로봇 제어 객체
        """
        self.node = node
        self.config_manager = config_manager
        self.robot_controller = robot_controller
        self.bridge = CvBridge() if node else None
        
        # 카메라 파라미터와 Hand-Eye 행렬 로드
        self.camera_matrix = None
        self.dist_coeffs = None
        self.hand_eye_matrix = None
        self._load_calibration_data()
        
        # ArUco 감지기 설정
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.aruco_params = aruco.DetectorParameters()
        
        # 이미지 구독자 (ROS2 노드가 있을 때만)
        self.image_sub = None
        self.current_image = None
        if self.node:
            self.image_sub = self.node.create_subscription(
                Image, '/camera/image_raw', self._image_callback, 10
            )
            
    def _load_calibration_data(self):
        """카메라 캘리브레이션과 Hand-Eye 행렬 로드"""
        try:
            # 카메라 파라미터 로드
            if self.config_manager:
                camera_file = self.config_manager.get_path('camera_params')
                hand_eye_file = self.config_manager.get_path('hand_eye_matrix')
            else:
                # fallback 경로
                from . import config
                camera_file = config.CAMERA_PARAMS_FILE
                hand_eye_file = config.HAND_EYE_MATRIX_FILE
                
            if os.path.exists(camera_file):
                camera_data = np.load(camera_file)
                self.camera_matrix = camera_data['camera_matrix']
                self.dist_coeffs = camera_data['dist_coeffs']
                
            if os.path.exists(hand_eye_file):
                self.hand_eye_matrix = np.load(hand_eye_file)
            else:
                # 기본 단위 행렬 사용
                self.hand_eye_matrix = np.eye(4)
                
        except Exception as e:
            print(f"캘리브레이션 데이터 로드 실패: {e}")
            # 기본값 설정
            self.camera_matrix = np.eye(3) * 500  # 대략적인 값
            self.dist_coeffs = np.zeros(5)
            self.hand_eye_matrix = np.eye(4)
            
    def _image_callback(self, msg: Image):
        """이미지 콜백 (ROS2 토픽에서 수신)"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"이미지 변환 실패: {e}")
                
    def detect_aruco_markers(self, image: Optional[np.ndarray] = None) -> Dict[int, Dict]:
        """
        ArUco 마커 감지
        
        Args:
            image: 입력 이미지 (None이면 현재 이미지 사용)
            
        Returns:
            {marker_id: {'corners': corners, 'pose': pose, 'distance': distance}}
        """
        if image is None:
            image = self.current_image
            
        if image is None:
            return {}
            
        # ArUco 마커 감지
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        results = {}
        
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                marker_corners = corners[i][0]  # shape: (4, 2)
                
                # 포즈 추정 (카메라 파라미터가 있을 때)
                pose_data = None
                distance = None
                
                if self.camera_matrix is not None:
                    # PnP로 포즈 추정
                    marker_size = 0.05  # 마커 크기 (m) - config에서 가져와야 함
                    object_points = np.array([
                        [-marker_size/2, marker_size/2, 0],
                        [marker_size/2, marker_size/2, 0],
                        [marker_size/2, -marker_size/2, 0],
                        [-marker_size/2, -marker_size/2, 0]
                    ], dtype=np.float32)
                    
                    success, rvec, tvec = cv2.solvePnP(
                        object_points, marker_corners,
                        self.camera_matrix, self.dist_coeffs
                    )
                    
                    if success:
                        # 카메라 좌표계에서 로봇 좌표계로 변환
                        camera_pose = np.eye(4)
                        camera_pose[:3, :3], _ = cv2.Rodrigues(rvec)
                        camera_pose[:3, 3] = tvec.flatten()
                        
                        # Hand-Eye 변환 적용
                        robot_pose = self.hand_eye_matrix @ camera_pose
                        
                        pose_data = {
                            'position': robot_pose[:3, 3],
                            'rotation': robot_pose[:3, :3],
                            'rvec': rvec,
                            'tvec': tvec
                        }
                        distance = np.linalg.norm(tvec)
                        
                results[marker_id] = {
                    'corners': marker_corners,
                    'pose': pose_data,
                    'distance': distance,
                    'center': np.mean(marker_corners, axis=0)
                }
                
        return results
    
    async def visual_servo_to_button(self, button_id: int, mode: str = 'oneshot') -> bool:
        """
        버튼까지 비전 서보잉 수행
        
        Args:
            button_id: 타겟 버튼 ID (ArUco 마커 ID)
            mode: 서보잉 모드 ('oneshot' 또는 'continuous')
            
        Returns:
            성공 여부
        """
        try:
            if mode == 'oneshot':
                return await self._oneshot_servo(button_id)
            else:
                # continuous 모드는 추후 구현
                return await self._oneshot_servo(button_id)
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"비전 서보잉 실패: {e}")
            return False
            
    async def _oneshot_servo(self, button_id: int) -> bool:
        """
        원샷 서보잉 - 한 번의 이미지로 목표 위치 계산 후 이동
        
        Args:
            button_id: 타겟 버튼 ID
            
        Returns:
            성공 여부
        """
        # ArUco 마커 감지
        markers = self.detect_aruco_markers()
        
        if button_id not in markers:
            if self.node:
                self.node.get_logger().warn(f"버튼 {button_id} 마커를 찾을 수 없습니다")
            return False
            
        marker_data = markers[button_id]
        
        if marker_data['pose'] is None:
            if self.node:
                self.node.get_logger().warn(f"버튼 {button_id} 포즈 추정 실패")
            return False
            
        # 목표 위치 계산 (마커 위치에서 약간 위쪽)
        target_position = marker_data['pose']['position'].copy()
        target_position[2] += 0.05  # 5cm 위쪽
        
        # IK로 관절 각도 계산 후 이동
        if self.robot_controller:
            return await self.robot_controller.move_to_position(target_position)
        else:
            if self.node:
                self.node.get_logger().error("robot_controller가 설정되지 않음")
            return False
            
    async def press_button(self, button_id: int) -> bool:
        """
        버튼 누르기 동작 수행
        
        Args:
            button_id: 버튼 ID
            
        Returns:
            성공 여부
        """
        try:
            # 1. 버튼 위로 이동
            if not await self.visual_servo_to_button(button_id, 'oneshot'):
                return False
                
            # 2. 약간 대기
            await asyncio.sleep(0.5)
            
            # 3. 버튼 누르기 (아래로 이동)
            markers = self.detect_aruco_markers()
            if button_id in markers and markers[button_id]['pose']:
                press_position = markers[button_id]['pose']['position'].copy()
                press_position[2] -= 0.02  # 2cm 아래로
                
                if self.robot_controller:
                    success = await self.robot_controller.move_to_position(press_position)
                    if success:
                        await asyncio.sleep(0.2)  # 버튼 누른 상태 유지
                        # 원래 위치로 복귀
                        press_position[2] += 0.07  # 5cm 위로
                        await self.robot_controller.move_to_position(press_position)
                    return success
                    
            return False
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"버튼 누르기 실패: {e}")
            return False
            
    def get_marker_pose_in_robot_frame(self, marker_id: int) -> Optional[Pose]:
        """
        마커의 로봇 좌표계 포즈 반환 (ROS 메시지 형태)
        
        Args:
            marker_id: 마커 ID
            
        Returns:
            Pose 메시지 또는 None
        """
        markers = self.detect_aruco_markers()
        
        if marker_id not in markers or markers[marker_id]['pose'] is None:
            return None
            
        pose_data = markers[marker_id]['pose']
        position = pose_data['position']
        rotation = pose_data['rotation']
        
        # 회전 행렬을 쿼터니언으로 변환 (scipy 없이 구현)
        def rotation_matrix_to_quaternion(R):
            """회전 행렬을 쿼터니언으로 변환"""
            trace = R[0, 0] + R[1, 1] + R[2, 2]
            if trace > 0:
                s = np.sqrt(trace + 1.0) * 2
                w = 0.25 * s
                x = (R[2, 1] - R[1, 2]) / s
                y = (R[0, 2] - R[2, 0]) / s
                z = (R[1, 0] - R[0, 1]) / s
            else:
                if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                    s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                    w = (R[2, 1] - R[1, 2]) / s
                    x = 0.25 * s
                    y = (R[0, 1] + R[1, 0]) / s
                    z = (R[0, 2] + R[2, 0]) / s
                elif R[1, 1] > R[2, 2]:
                    s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                    w = (R[0, 2] - R[2, 0]) / s
                    x = (R[0, 1] + R[1, 0]) / s
                    y = 0.25 * s
                    z = (R[1, 2] + R[2, 1]) / s
                else:
                    s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                    w = (R[1, 0] - R[0, 1]) / s
                    x = (R[0, 2] + R[2, 0]) / s
                    y = (R[1, 2] + R[2, 1]) / s
                    z = 0.25 * s
            return np.array([x, y, z, w])
        
        quat = rotation_matrix_to_quaternion(rotation)  # [x, y, z, w]
        
        # ROS Pose 메시지 생성
        pose = Pose()
        pose.position = Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))
        pose.orientation = Quaternion(x=float(quat[0]), y=float(quat[1]), 
                                    z=float(quat[2]), w=float(quat[3]))
        
        return pose
    
    async def _oneshot_servo(self, button_id: int) -> bool:
        """한 번에 목표 위치로 이동"""
        # 버튼 3D 위치 감지
        button_transform = await self._detect_button_pose(button_id)
        if button_transform is None:
            self._log("버튼 위치 감지 실패", error=True)
            return False
        
        # 버튼 위치와 방향 추출
        button_pos = button_transform[:3, 3]
        button_orientation = button_transform[:3, :3]
        button_z_vector = button_orientation[:, 2]
        
        # 준비 위치 계산 (버튼 앞 일정 거리)
        standby_pos = button_pos - button_z_vector * config.SERVOING_STANDBY_DISTANCE_M
        
        # 현재 위치에서 버튼을 바라보는 방향 계산
        current_pos = self._get_current_end_effector_pos()
        target_orientation = self._create_look_at_matrix(current_pos, button_pos)
        
        if target_orientation is None:
            self._log("Look-at 방향 계산 실패", error=True)
            return False
        
        # 디버그 마커 표시
        if self.node:
            self._publish_debug_markers(button_pos, standby_pos)
        
        # 준비 위치로 이동
        self._log(f"준비 위치로 이동: {np.round(standby_pos, 3)}")
        success = await self.robot.move_to_pose_ik(standby_pos, target_orientation, blocking=True)
        
        if success:
            self.last_target_pose = standby_pos
            self.last_target_orientation = target_orientation
            self._log("✅ 준비 위치 정렬 성공")
        
        return success
    
    async def _iterative_servo(self, button_id: int) -> bool:
        """반복적 정렬 (기존 align_to_standby_pose 로직)"""
        self._log(f"반복 서보잉 시작: 버튼 ID {button_id}")
        
        for attempt in range(self.max_attempts):
            self._log(f"--- 정렬 시도 #{attempt + 1} ---")
            
            # 현재 버튼 위치 감지
            button_transform = await self._detect_button_pose(button_id)
            if button_transform is None:
                await asyncio.sleep(0.1)
                continue
            
            button_pos = button_transform[:3, 3]
            button_orientation = button_transform[:3, :3]
            button_z_vector = button_orientation[:, 2]
            standby_pos = button_pos - button_z_vector * config.SERVOING_STANDBY_DISTANCE_M
            
            current_pos = self._get_current_end_effector_pos()
            position_error_vec = standby_pos - current_pos
            position_error = np.linalg.norm(position_error_vec)
            
            self._log(f"준비 위치까지 거리: {position_error * 1000:.2f} mm")
            
            # 허용 오차 내 도달 확인
            if position_error < config.SERVOING_POSITION_TOLERANCE_M:
                self.last_target_pose = standby_pos
                self.last_target_orientation = button_orientation
                self._log("✅ 준비 위치 정렬 성공")
                return True
            
            # 스텝 이동
            move_direction = position_error_vec / (position_error + 1e-6)
            step_distance = min(config.SERVOING_MAX_STEP_M, position_error)
            
            if step_distance < config.IK_MIN_STEP_M:
                self._log("이동 거리가 최소 스텝보다 작아 성공으로 간주")
                return True
            
            step_target_xyz = current_pos + move_direction * step_distance
            target_orientation = self._create_look_at_matrix(current_pos, button_pos)
            
            if target_orientation is None:
                self._log("Look-at 방향 계산 실패", error=True)
                continue
            
            # 한 스텝 이동
            if not await self.robot.move_to_pose_ik(step_target_xyz, target_orientation, blocking=True):
                self._log("IK 이동 스텝 실패", error=True)
                return False
            
            await asyncio.sleep(0.1)
        
        self._log(f"❌ 최대 시도 횟수({self.max_attempts}회) 내 정렬 실패", error=True)
        return False
    
    async def _detect_button_pose(self, button_id: int) -> np.ndarray:
        """버튼 3D 위치 감지"""
        if not self.vision_client:
            self._log("VisionClient가 초기화되지 않았습니다", error=True)
            return None
        
        try:
            # 비전 서비스 요청
            response = await self.vision_client.request_status(
                mode=config.POSE_ESTIMATION_MODE,
                robot_id=self.robot_id,
                button_id=button_id
            )
            
            if not response or not response.success:
                self._log("버튼 시야 이탈 또는 감지 실패", error=True)
                return None
            
            # 현재 로봇 FK 변환 행렬 계산
            current_robot_transform = self._get_current_robot_transform()
            
            # 좌표 변환
            if config.POSE_ESTIMATION_MODE == 'corner':
                if hasattr(response, 'corners') and len(response.corners) == 8:
                    image_points_2d = np.array(response.corners, dtype=np.float32).reshape((4, 2))
                    return self.coord_transformer.get_button_pose_in_base_frame(
                        robot_fk_transform=current_robot_transform,
                        mode='corner',
                        image_points_2d=image_points_2d
                    )
            elif config.POSE_ESTIMATION_MODE == 'normal':
                return self.coord_transformer.get_button_pose_in_base_frame(
                    robot_fk_transform=current_robot_transform,
                    mode='normal',
                    center_x_norm=response.x,
                    center_y_norm=response.y,
                    size_norm=response.size
                )
            
            self._log(f"지원하지 않는 모드: {config.POSE_ESTIMATION_MODE}", error=True)
            return None
            
        except Exception as e:
            self._log(f"버튼 감지 오류: {e}", error=True)
            return None
    
    def _get_current_end_effector_pos(self) -> np.ndarray:
        """현재 엔드 이펙터 위치 반환"""
        current_transform = self._get_current_robot_transform()
        return current_transform[:3, 3]
    
    def _get_current_robot_transform(self) -> np.ndarray:
        """현재 로봇 FK 변환 행렬 계산"""
        current_angles = self.robot.get_current_angles_rad()
        full_joints = self.robot.kin_solver._get_full_joints(current_angles)
        return self.robot.kin_solver.chain.forward_kinematics(full_joints)
    
    def _create_look_at_matrix(self, camera_pos: np.ndarray, target_pos: np.ndarray, 
                              world_up: np.ndarray = np.array([0, 0, 1])) -> np.ndarray:
        """카메라 위치에서 목표 위치를 바라보는 회전 행렬 생성"""
        epsilon = 1e-6
        
        forward = target_pos - camera_pos
        forward_norm = np.linalg.norm(forward)
        if forward_norm < epsilon:
            return None
        forward /= forward_norm
        
        if abs(np.dot(forward, world_up)) > 0.999:
            right = np.cross(np.array([1, 0, 0]), forward)
        else:
            right = np.cross(world_up, forward)
        
        right_norm = np.linalg.norm(right)
        if right_norm < epsilon:
            return None
        right /= right_norm
        
        down = np.cross(forward, right)
        return np.array([right, down, forward]).T
    
    def _publish_debug_markers(self, button_pos: np.ndarray, standby_pos: np.ndarray):
        """RViz 디버그 마커 퍼블리시"""
        if not self.node:
            return
        
        # 버튼 위치 마커 (빨간색)
        marker_btn = Marker()
        marker_btn.header.frame_id = "base_link"
        marker_btn.header.stamp = self.node.get_clock().now().to_msg()
        marker_btn.ns = "targets"
        marker_btn.id = 0
        marker_btn.type = Marker.SPHERE
        marker_btn.action = Marker.ADD
        marker_btn.pose.position = Point(x=button_pos[0], y=button_pos[1], z=button_pos[2])
        marker_btn.pose.orientation.w = 1.0
        marker_btn.scale.x = 0.03
        marker_btn.scale.y = 0.03
        marker_btn.scale.z = 0.03
        marker_btn.color.a = 0.8
        marker_btn.color.r = 1.0
        
        # 준비 위치 마커 (파란색)
        marker_sby = Marker()
        marker_sby.header = marker_btn.header
        marker_sby.ns = marker_btn.ns
        marker_sby.id = 1
        marker_sby.type = Marker.SPHERE
        marker_sby.action = Marker.ADD
        marker_sby.pose.position = Point(x=standby_pos[0], y=standby_pos[1], z=standby_pos[2])
        marker_sby.pose.orientation.w = 1.0
        marker_sby.scale = marker_btn.scale
        marker_sby.color.a = 0.8
        marker_sby.color.b = 1.0
        
        self.marker_pub.publish(marker_btn)
        self.marker_pub.publish(marker_sby)
    
    async def press_button(self) -> bool:
        """버튼 누르기 동작"""
        if self.last_target_pose is None or self.last_target_orientation is None:
            self._log("준비 위치 정보가 없습니다. 먼저 정렬을 수행하세요.", error=True)
            return False
        
        # 버튼을 누르기 위해 앞으로 이동
        press_direction = self.last_target_orientation[:, 2]  # Z축 방향 (정면)
        press_target = self.last_target_pose + press_direction * config.PRESS_FORWARD_DISTANCE_M
        
        self._log(f"버튼 누르기: {np.round(press_target, 3)}")
        
        # 버튼 누르기
        if not await self.robot.move_to_pose_ik(press_target, self.last_target_orientation, blocking=True):
            self._log("버튼 누르기 실패", error=True)
            return False
        
        await asyncio.sleep(0.5)  # 잠시 대기
        
        # 준비 위치로 복귀
        self._log("준비 위치로 복귀")
        if not await self.robot.move_to_pose_ik(self.last_target_pose, self.last_target_orientation, blocking=True):
            self._log("복귀 실패", error=True)
            return False
        
        self._log("✅ 버튼 누르기 완료")
        return True
    
    def _log(self, message: str, error: bool = False):
        """로그 출력"""
        if config.DEBUG:
            log_level = "ERROR" if error else "INFO"
            print(f"[VisionController][{log_level}] {message}")
