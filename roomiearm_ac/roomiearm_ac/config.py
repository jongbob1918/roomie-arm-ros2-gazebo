from pathlib import Path
import numpy as np
from enum import IntEnum
import os


# 1. Pose 종류를 관리하는 Enum 클래스 정의
class Pose(IntEnum):
    """로봇 팔의 미리 정의된 자세 종류"""
    INIT = 0
    OBSERVE = 1
    LEFT = 2
    RIGHT = 3
    FORWARD = 4
    UP= 5
    UPUP = 6

# 버튼 피드백 상태PBVS_PRESS
class ButtonActionStatus:
    MOVING_TO_TARGET = "MOVING_TO_TARGET"
    ALIGNING_TO_TARGET = "ALIGNING_TO_TARGET"
    PRESSING = "PRESSING"
    RETRACTING = "RETRACTING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"

class ControlStrategy(IntEnum):
    """
    로봇팔의 전체적인 행동 전략을 결정합니다.
    1. 즉시 누르기: 사전 정의된 좌표로 즉시 이동하여 누릅니다. (동기)
    2. 관측 후 누르기: 사전 정의된 좌표의 5cm 앞에서 멈춘 후, 누릅니다. (동기)
    3. 시각 서보잉: 카메라를 이용해 버튼을 추적하여 누릅니다. (비동기/동기 혼합)
    """
    MODEL_DIRECT_PRESS = 1
    MODEL_STANDBY_PRESS = 2
    PBVS_PRESS = 3
    PBVS_ONESHOT_PRESS = 4


# 제어 전략 설정
# ControlMode.MODEL_ONLY : 미리 정의된 좌표로 이동 (이미지 서보잉 없음)
# ControlMode.HYBRID     : 비전 기반 이미지 서보잉 사용 (기존 방식)
CONTROL_STRATEGY = ControlStrategy.PBVS_PRESS

# --- [추가] 좌표 계산 모드 설정 ---
# 'corner': ArUco 마커의 4개 모서리점을 이용한 PnP 계산 (정확도 높음)
# 'normal': 중심점과 크기만을 이용한 거리 추정 계산 (정확도 낮음, fallback용)
POSE_ESTIMATION_MODE = 'normal' # 'corner' 또는 'normal'로 변경하여 테스트 가능

# 기본 설정 (DEBUG 등)
DEBUG = True

# True일 경우, PBVS/IBVS 제어 시 시야에서 계산된 방향 대신 로봇 베이스와 정렬된 안정적인 방향을 사용합니다.
USE_STABLE_ORIENTATION = True 

# True일 경우, Hand-Eye 보정 행렬을 무시하고 단위 행렬을 사용합니다 (디버깅용).
IGNORE_HAND_EYE_CALIBRATION = True



# 파일 경로 설정
SCRIPT_DIR = Path(__file__).resolve().parent
# 패키지 경로 의존성 제거 (테스트용)
DATA_DIR = os.path.join(SCRIPT_DIR, '..', 'data')
URDF_FILE = os.path.join(SCRIPT_DIR, '..', 'urdf', 'roomie2.urdf')
CAMERA_PARAMS_FILE = os.path.join(DATA_DIR, 'camera_params.npz')  
HAND_EYE_MATRIX_FILE = os.path.join(DATA_DIR, 'hand_eye_matrix.npy')
# 시리얼 통신 설정
SERIAL_PORT = "/dev/ttyUSB0" 
SERIAL_BAUD_RATE = 460800
SERIAL_TIMEOUT = 10.0 # 시리얼 연결 및 응답 대기 타임아웃

# --- 카메라 및 인식 설정 ---
CAMERA_DEVICE_ID = 8  # 사용자의 카메라 장치 번호

# YOLO 모델 경로 (패키지 내 상대 경로)
import os
from ament_index_python.packages import get_package_share_directory

try:
    package_share_directory = get_package_share_directory('roomie_ac')
    YOLO_MODEL_PATH = os.path.join(package_share_directory, 'data', 'best.pt')
except Exception:
    # 개발 환경에서는 상대 경로 사용
    current_dir = os.path.dirname(__file__)
    YOLO_MODEL_PATH = os.path.join(current_dir, 'data', 'best.pt')



# 서보 모터 및 관절 설정
SERVO_ZERO_OFFSET_DEG = np.array([90, 90, 90, 90])
SERVO_DIRECTION_MULTIPLIER = np.array([1, 1, 1, 1])
JOINT_LIMIT_DEG = np.array([[-90, 90], [-90, 90], [-90, 90], [-90, 90]])
JOINT_LIMIT_RAD = np.deg2rad(JOINT_LIMIT_DEG) # 라디안 변환
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4'] # RViz2 퍼블리싱을 위한 관절 이름

# IK (Inverse Kinematics) 설정
ACTIVE_LINKS_MASK = [False, True, True, True, True, False] # ikpy 체인에서 활성화할 링크 마스크
IK_MAX_ITERATIONS = 20000 # IK 최대MODEL_DIRECT_PRESS 반복 횟수
IK_TOLERANCE_M = 1e-3 # IK 오차 허용 범위 (m)



# 워크스페이스 (작업 공간) 설정
WORKSPACE_R_MIN_M = -0.45 # 로봇 중심으로부터 최소/최대 반경 (m)PBVS_PRESS
WORKSPACE_R_MAX_M = 0.45
WORKSPACE_Z_MIN_M = -0.40 # 로봇팔 끝점의 최소/최대 높이 (m)
WORKSPACE_Z_MAX_M = 0.402

# 동작 및 지연 시간 설정
COMEBACK_DELAY_SEC = 20.0 # 홈 포지션으로 복귀 대기 시간 (초)



# 2. Enum을 키(key)로, 실제 각도값을 값(value)으로 갖는 딕셔너리 생성
POSE_ANGLES_DEG = {
    Pose.INIT: np.array([0, 40, 170, 30]),
    Pose.OBSERVE: np.array([90, 140, 168, 30]),
    Pose.LEFT: np.array([170, 120, 150, 30]),
    Pose.RIGHT: np.array([0, 130, 170, 30]),
    Pose.FORWARD: np.array([90, 140, 170, 30]),
    Pose.UP: np.array([90, 120, 137, 30]),
    Pose.UPUP: np.array([85, 130, 137, 30]),

}
# 기존 홈 포지션 변수도 이 딕셔너리를 활용할 수 있습니다.
HOME_POSITION_SERVO_DEG = POSE_ANGLES_DEG[Pose.INIT]



# --- 로봇 및 버튼 ---
ROBOT_ID = 0

# 사전 정의된 버튼 위치 (미터 단위, MODEL_ONLY 모드용)
PREDEFINED_BUTTON_POSES_M = {
    0: np.array([0.235, 0.0, 0.305]),  # 예시 좌표 (button_id: 2)
    1: np.array([0.135, 0.10, 0.15]),  
    2: np.array([0.135, -0.10, 0.15]), 
    3: np.array([-0.135, 0.0, 0.15]),  
    4: np.array([0.135, 0.0, 0.15]),  
    5: np.array([0.235, 0.0, 0.235]), 
    6: np.array([0.245, 0.0, 0.215]),  
    101: np.array([0.25, 0.0, 0.29]), 
    102: np.array([0.235, 0.0,0.095]),  
    # 다른 버튼 ID와 좌표를 여기에 추가할 수 있습니다.
    # 3: np.array([0.25, -0.1, 0.15]),
}

REAL_BUTTON_DIAMETER_M = 0.035 # 3.5cm
# --- 비전 ---

IMAGE_WIDTH_PX = 800
IMAGE_HEIGHT_PX = 600
HAND_EYE_UNIT = 'mm' # 핸드-아이 보정 시 사용한 단위, 'mm' 또는 'm'

# PnP를 위한 원형 버튼의 3D 모델 포인트 (대칭성 보장)
OBJECT_POINTS_3D = np.array([
    [ REAL_BUTTON_DIAMETER_M / 2,  0.0, 0.0], # 버튼 오른쪽
    [-REAL_BUTTON_DIAMETER_M / 2,  0.0, 0.0], # 버튼 왼쪽
    [ 0.0,  REAL_BUTTON_DIAMETER_M / 2, 0.0], # 버튼 위쪽
    [ 0.0, -REAL_BUTTON_DIAMETER_M / 2, 0.0], # 버튼 아래쪽
], dtype=np.float32)

# PnP RANSAC 파라미터
PNPR_REPROJ_ERROR_THRESHOLD_PX = 8.0
PNPR_MIN_INLIERS = 3

# --- 이미지 서보잉 ---
SERVOING_POSITION_TOLERANCE_M = 0.002
SERVOING_STANDBY_DISTANCE_M = 0.08 #  목표 조준을 위한 안전거리
PRESS_FORWARD_DISTANCE_M = 0.1 # 실제 버튼을 누르는 이동 거리
SERVOING_MAX_STEP_M = 0.03 # 멀리서 버튼에 접근할 때의 최대 이동 스텝 (2cm)
IK_MIN_STEP_M = 0.005 # 5mm   이 거리보다 짧은 이동은 무시
