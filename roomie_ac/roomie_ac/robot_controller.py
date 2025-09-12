"""
통합 로봇 제어 클래스 - 시뮬레이션/실물 모드 지원
"""
import serial
import time
import numpy as np
import asyncio
from . import config
from .kinematics_solver import KinematicsSolver

# Gazebo용 ROS2 imports (조건부)
try:
    import rclpy
    from std_msgs.msg import Float64MultiArray, Float64
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class RobotController:
    """단일 클래스로 모든 로봇 제어 기능 통합"""
    
    def __init__(self, simulation_mode: bool = False, node=None):
        """
        Args:
            simulation_mode: True=Gazebo, False=실물 로봇
            node: ROS2 노드 (Gazebo 모드에서 필요)
        """
        self.simulation_mode = simulation_mode
        self.node = node
        self.kin_solver = KinematicsSolver()
        self.current_angles_rad = self._convert_servo_deg_to_rad(np.array(config.HOME_POSITION_SERVO_DEG))
        
        # 하드웨어 초기화
        if simulation_mode:
            self._setup_simulation_mode()
        else:
            self._setup_real_mode()
    
    def _setup_simulation_mode(self):
        """Gazebo 모드 설정 - arm_controller 사용"""
        if not ROS2_AVAILABLE or self.node is None:
            self._log("Gazebo 모드 설정 실패: ROS2 메시지 또는 노드가 없습니다.", error=True)
            return
        
        # arm_controller 사용 (joint_trajectory_controller)
        # ros2_control Forward Position Controller 퍼블리셔 (시뮬레이션용)
        self.position_command_pub = self.node.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            10
        )
        
        # 조인트 상태 수신
        self.joint_state_sub = self.node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        
        self._log("✅ Gazebo 모드 활성화 - arm_controller 사용")
    
    def _setup_real_mode(self):
        """실물 로봇 모드 설정 - ESP32 시리얼"""
        self.serial_port = config.SERIAL_PORT
        self.serial_baud = config.SERIAL_BAUD_RATE
        self.serial_timeout = config.SERIAL_TIMEOUT
        self.ser = None
        
        # 시리얼 연결
        if not self._connect_serial():
            raise ConnectionError("ESP32 시리얼 연결 실패")
        
        self._log("✅ 실물 로봇 모드 활성화 - ESP32 시리얼 사용")
    
    def _connect_serial(self) -> bool:
        """ESP32 시리얼 연결"""
        try:
            self.ser = serial.Serial(self.serial_port, self.serial_baud, timeout=self.serial_timeout)
            self._log("🔌 시리얼 포트 연결됨. ESP32 부팅 대기 중")
            time.sleep(2)
            
            # 홈 포지션으로 이동
            success = self._send_servo_command(config.HOME_POSITION_SERVO_DEG)
            if success:
                self._log("🤝 ESP32와 연결 성공 및 홈 포지션 이동 완료")
                return True
            else:
                self._log("❌ ESP32 초기 명령 전송 실패", error=True)
                return False
        except serial.SerialException as e:
            self._log(f"❌ 시리얼 연결 실패: {e}", error=True)
            return False
    
    async def move_to_angles_rad(self, angles_rad: np.ndarray, blocking: bool = True) -> bool:
        """관절 각도로 이동 (라디안)"""
        self._log(f"관절 각도로 이동: {np.round(angles_rad, 3)} (blocking: {blocking})")
        
        if self.simulation_mode:
            return self._send_to_gazebo(angles_rad)
        else:
            return await self._send_to_real_robot(angles_rad, blocking)
    
    async def move_to_pose_ik(self, xyz: np.ndarray, orientation=None, blocking: bool = True) -> bool:
        """3D 좌표로 이동 (IK 사용)"""
        self._log(f"IK 이동: 좌표 {np.round(xyz, 3)}, 방향: {'자동' if orientation is None else '지정됨'}")
        
        # IK 계산
        solution_rad = self.kin_solver.solve_ik(xyz, orientation, self.current_angles_rad)
        if solution_rad is None:
            self._log("IK 해를 찾지 못했습니다.", error=True)
            return False
        
        return await self.move_to_angles_rad(solution_rad, blocking)
    
    def get_current_angles_rad(self) -> np.ndarray:
        """현재 관절 각도 반환 (라디안)"""
        return self.current_angles_rad.copy()
    
    def _send_to_gazebo(self, angles_rad: np.ndarray) -> bool:
        """ros2_control Forward Position Controller로 명령 전송"""
        try:
            # Float64MultiArray 메시지 생성
            command_msg = Float64MultiArray()
            
            # 4DOF 각도 설정
            command_msg.data = angles_rad.tolist()
            
            # 명령 전송
            self.position_command_pub.publish(command_msg)
            
            # 상태 업데이트
            self.current_angles_rad = angles_rad
            self._log(f"ros2_control로 전송 완료: {np.round(angles_rad, 3)}")
            return True
            
        except Exception as e:
            self._log(f"ros2_control 전송 실패: {e}", error=True)
            return False
            return False
    
    async def _send_to_real_robot(self, angles_rad: np.ndarray, blocking: bool) -> bool:
        """실물 로봇으로 시리얼 명령 전송"""
        # 라디안을 서보 각도로 변환
        servo_angles_deg = self._convert_rad_to_servo_deg(angles_rad)
        
        if not self._send_servo_command(servo_angles_deg):
            self._log("ESP32 명령 전송 실패", error=True)
            return False
        
        if blocking:
            if not await self._wait_for_ack():
                self._log("ESP32 ACK 대기 시간 초과", error=True)
                return False
        
        # 상태 업데이트
        self.current_angles_rad = angles_rad
        return True
    
    def _send_servo_command(self, angles_deg: np.ndarray) -> bool:
        """ESP32로 서보 명령 전송"""
        if not self.ser or not self.ser.is_open:
            return False
        
        int_angles = np.round(angles_deg).astype(int)
        cmd = f"<M:{','.join(map(str, int_angles))}>"
        
        try:
            self.ser.write(cmd.encode('utf-8'))
            if config.DEBUG:
                self._log(f"[SERIAL TX] -> {cmd}")
            return True
        except serial.SerialException as e:
            self._log(f"시리얼 전송 오류: {e}", error=True)
            return False
    
    async def _wait_for_ack(self, timeout: float = None) -> bool:
        """ESP32 ACK 신호 대기"""
        if timeout is None:
            timeout = config.SERIAL_TIMEOUT
            
        if not self.ser or not self.ser.is_open:
            return False
        
        self.ser.reset_input_buffer()
        start_time = time.time()
        buffer = b''
        
        while time.time() - start_time < timeout:
            if self.ser.in_waiting > 0:
                buffer += self.ser.read(self.ser.in_waiting)
                if b'<D>' in buffer:
                    if config.DEBUG:
                        self._log("[SERIAL RX] -> <D> (ACK 수신)")
                    return True
            await asyncio.sleep(0.01)
        
        return False
    
    def _joint_state_callback(self, msg: JointState):
        """Gazebo 조인트 상태 콜백"""
        try:
            # arm_description xacro의 조인트명 사용
            joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
            angles = []
            
            for joint_name in joint_names:
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    angles.append(msg.position[idx])
            
            if len(angles) == 4:
                self.current_angles_rad = np.array(angles)
                if config.DEBUG:
                    self._log(f"Gazebo 조인트 상태: {np.round(self.current_angles_rad, 3)}")
                    
        except Exception as e:
            self._log(f"조인트 상태 콜백 오류: {e}", error=True)
    
    def _convert_rad_to_servo_deg(self, angles_rad: np.ndarray) -> np.ndarray:
        """라디안을 서보 각도로 변환"""
        angles_deg_ik = np.rad2deg(angles_rad)
        servo_angles = config.SERVO_ZERO_OFFSET_DEG + angles_deg_ik * config.SERVO_DIRECTION_MULTIPLIER
        return np.round(servo_angles).astype(int)
    
    def _convert_servo_deg_to_rad(self, angles_deg: np.ndarray) -> np.ndarray:
        """서보 각도를 라디안으로 변환"""
        angles_deg_ik = (angles_deg - config.SERVO_ZERO_OFFSET_DEG) * config.SERVO_DIRECTION_MULTIPLIER
        return np.deg2rad(angles_deg_ik)
    
    def disconnect(self):
        """연결 해제"""
        if not self.simulation_mode and self.ser:
            if self.ser.is_open:
                self.ser.close()
            self._log("ESP32 시리얼 연결 해제됨")
            
    async def move_to_position(self, position: np.ndarray, blocking: bool = True) -> bool:
        """
        3D 위치로 이동 (IK 사용) - vision_controller에서 호출용
        
        Args:
            position: [x, y, z] 좌표 (m)
            blocking: 동작 완료 대기 여부
            
        Returns:
            성공 여부
        """
        return await self.move_to_pose_ik(position, blocking=blocking)
    
    def _log(self, message: str, error: bool = False):
        """로그 출력"""
        if config.DEBUG:
            log_level = "ERROR" if error else "INFO"
            print(f"[RobotController][{log_level}] {message}")
