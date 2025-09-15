RoomieArm 리팩토링 계획서
📋 현재 상태 분석
주요 문제점

IK/좌표계 변환 불안정성: ikpy와 수동 행렬 계산으로 인한 정확도 문제
아키텍처 일관성 부족: vision_node는 MarkerArray를 사용하지만, 설계서는 MarkerDetection을 언급
카메라 해상도 불일치: URDF(800x600) vs 코드(640x480)
에러 처리 미흡: TF2, IK 실패 시 복구 메커니즘 부재
하드코딩된 설정값: 카메라 파라미터, 마커 크기 등이 코드에 하드코딩

현재 완성도

✅ Phase 1 완료: ros2_control 및 하드웨어 인터페이스
⚠️ Phase 2 부분완료: vision_node 구현됨, button_click_node 개선 필요
❌ Phase 3 미완료: 시스템 통합 및 런치 파일

🎯 리팩토링 목표

안정적인 IK 솔버 구현: 기하학적 접근법으로 정확도 향상
모듈화된 아키텍처: 각 컴포넌트의 책임 명확화
설정 중앙화: YAML 파일을 통한 파라미터 관리
강화된 에러 처리: 실패 상황 대응 메커니즘 구축
테스트 가능한 구조: 단위/통합 테스트 지원

📅 단계별 리팩토링 계획
🔧 Phase 1: 핵심 인프라 개선 (우선순위: 높음)
1.1 메시지 타입 통일 및 설정 중앙화

작업 내용:

MarkerArray vs MarkerDetection 메시지 타입 결정
카메라 파라미터, 로봇 파라미터를 YAML 설정 파일로 분리
해상도 불일치 문제 해결 (URDF와 코드 동기화)


예상 소요시간: 1일
우선 작업:

yaml  # config/camera_params.yaml
  camera:
    width: 800
    height: 600
    matrix: [615.4, 0.0, 400.0, 0.0, 615.4, 300.0, 0.0, 0.0, 1.0]
    distortion_coeffs: [0.0, 0.0, 0.0, 0.0]
  
  aruco:
    marker_size: 0.035
    dictionary: DICT_4X4_250
1.2 개선된 KinematicsSolver 구현

현재 문제: ikpy 기반으로 불안정한 결과
해결 방안: 4-DOF 로봇팔의 기하학적 특성을 활용한 직접 계산
구현 방향:

python  class KinematicsSolver:
      def __init__(self, robot_params):
          # DH 파라미터 로드
          self.dh_params = robot_params['dh_parameters']
          
      def forward_kinematics(self, joint_angles):
          # 순기구학 계산
          
      def inverse_kinematics(self, target_position, target_orientation=None):
          # 기하학적 역기구학 계산
          # 여러 해가 존재할 경우 현재 자세와 가장 가까운 해 선택
🎯 Phase 2: 비전 및 제어 로직 개선 (우선순위: 높음)
2.1 VisionNode 리팩토링

개선 사항:

설정 파일에서 카메라 파라미터 로드
마커 감지 신뢰성 향상 (연속 프레임 검증)
성능 최적화 (불필요한 계산 제거)



2.2 ButtonClickNode 핵심 로직 개선

PBVS 계산 안정화:

python  def estimate_marker_pose(self, marker_corners, camera_matrix, dist_coeffs):
      # solvePnP 결과 검증 로직 추가
      # 여러 알고리즘 시도 (SOLVEPNP_ITERATIVE, SOLVEPNP_SQPNP 등)
      
  def transform_pose_to_base(self, camera_pose, camera_frame):
      # TF2 변환 실패 시 재시도 메커니즘
      # 변환 결과 유효성 검증
2.3 동작 시퀀스 개선

현재: 하드코딩된 오프셋 (5cm, 0.5cm)
개선: 마커 법선 벡터 기반 접근 방향 계산
안전성: 충돌 감지 및 회피 로직 추가

🔄 Phase 3: 시스템 통합 및 테스트 (우선순위: 중간)
3.1 런치 시스템 구축
python# launch/roomie_bringup.launch.py
def generate_launch_description():
    # 시뮬레이션/실제 환경 선택 로직
    # 설정 파일 자동 로드
    # 노드 간 의존성 관리
3.2 에러 처리 및 복구 메커니즘

타임아웃 관리: 각 단계별 제한 시간 설정
실패 복구: IK 실패 시 대안 자세 시도
안전 정지: 예상치 못한 상황에서 로봇 정지

🧪 Phase 4: 테스트 및 검증 (우선순위: 중간)
4.1 단위 테스트 구축
python# tests/test_kinematics_solver.py
class TestKinematicsSolver(unittest.TestCase):
    def test_ik_accuracy(self):
        # IK -> FK 일관성 검증
        
    def test_workspace_limits(self):
        # 작업 공간 경계 테스트
4.2 통합 테스트 시나리오

시뮬레이션 환경: 가상 환경에서의 전체 워크플로우
실제 환경: 실물 로봇에서의 정확도 검증

⚡ 즉시 수행 가능한 개선 작업
1️⃣ 긴급 수정 (30분 내)

 URDF 카메라 해상도를 640x480으로 수정
 button_click_node의 하드코딩된 이미지 크기 변수화
 로그 메시지 일관성 개선

2️⃣ 설정 파일 분리 (2시간 내)

 config/ 폴더 생성 및 YAML 파일 작성
 노드들이 설정 파일에서 파라미터 로드하도록 수정

3️⃣ 에러 처리 강화 (4시간 내)

 TF2 변환 실패 시 재시도 로직 추가
 solvePnP 실패 시 대안 알고리즘 시도
 각 단계별 타임아웃 설정

📊 예상 효과
개선 전 vs 개선 후 비교
항목개선 전개선 후IK 정확도불안정 (ikpy 의존)안정적 (기하학적 계산)설정 관리하드코딩YAML 중앙화에러 대응단순 실패복구 메커니즘테스트 가능성어려움모듈화로 용이유지보수복잡함명확한 책임 분리
🚀 실행 순서 권장사항

1일차: Phase 1.1 (설정 중앙화) + 긴급 수정
2-3일차: Phase 1.2 (KinematicsSolver 재구현)
4-5일차: Phase 2 (비전 및 제어 로직 개선)
6일차: Phase 3 (시스템 통합)
7일차: Phase 4 (테스트 및 검증)

💡 추가 제안사항
장기 개선 방향

GUI 도구: 마커 위치 시각화 및 캘리브레이션 도구
동적 파라미터: 런타임에 설정 변경 가능
성능 모니터링: 각 단계별 실행 시간 및 성공률 추적
안전 기능: 충돌 감지, 비상 정지 등

개발 도구 활용

디버깅: rqt_graph, ros2 topic echo 적극 활용
시각화: rviz2를 통한 좌표계 및 궤적 확인
프로파일링: Python cProfile로 성능 병목 지점 파악

이 계획을 따라 진행하시면, 현재의 불안정한 시스템을 견고하고 확장 가능한 아키텍처로 개선할 수 있습니다!



-----







# RoomieArm 리팩토링 설계 문서

## 개요
ROS2 Jazzy + Gazebo Harmonic 환경에서 4-DOF 로봇팔의 벽면 ArUco 마커 버튼 클릭 시스템 리팩토링

---

## 1. 시스템 아키텍처

### 좌표계 변환 체인
```
Camera Detection → TF2 Transform → IK Calculation → Joint Control
```

**세부 단계:**
1. `solvePnP` → camera_link 기준 마커 3D 위치
2. `camera_link → ee_link` → TF2 변환
3. `ee_link → base_link` → TF2 변환  
4. `IK Solver` → 조인트 각도 계산
5. `ros2_control` → 로봇 구동

---

## 2. button_click_node.py 리팩토링

### 2.1 카메라 파라미터 수정
```python
# 실제 캘리브레이션 데이터 (640x480 해상도)
self.camera_matrix = np.array([
    [693.32, 0, 300.69],
    [0, 692.29, 281.98],
    [0, 0, 1]], dtype=np.float32)

self.dist_coeffs = np.array([
    -0.4149, 0.2780, 0.0004, -0.0002, -0.1941
], dtype=np.float32)

self.image_width_px = 640
self.image_height_px = 480
```

### 2.2 좌표계 변환 로직 개선
```python
def transform_marker_to_base(self, marker_pose_camera):
    """2단계 TF2 변환: camera_link → ee_link → base_link"""
    
    # Step 1: camera_link → ee_link
    try:
        transform_cam_to_ee = self.tf_buffer.lookup_transform(
            'ee_link', 'camera_link', 
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=1.0)
        )
        marker_pose_ee = do_transform_pose_stamped(
            marker_pose_camera, transform_cam_to_ee
        )
    except Exception as e:
        self.get_logger().error(f'Camera to EE transform failed: {e}')
        return None, None
    
    # Step 2: ee_link → base_link
    try:
        transform_ee_to_base = self.tf_buffer.lookup_transform(
            'base_link', 'ee_link',
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=1.0)
        )
        marker_pose_base = do_transform_pose_stamped(
            marker_pose_ee, transform_ee_to_base
        )
        return marker_pose_ee, marker_pose_base
    except Exception as e:
        self.get_logger().error(f'EE to Base transform failed: {e}')
        return marker_pose_ee, None
```

### 2.3 벽면 접근 자세 계산
```python
def calculate_wall_approach_poses(self, marker_pose_base):
    """벽면 마커를 위한 접근 자세 계산 (원추형 접근 영역)"""
    
    # 마커 법선 벡터 (벽에서 로봇 방향)
    orientation = marker_pose_base.pose.orientation
    r = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
    marker_normal = r.as_matrix()[:, 2]  # z축 = 법선
    
    # 기본 접근 방향 (법선의 반대)
    ideal_approach = -marker_normal
    
    # 원추형 영역에서 접근 방향 후보 생성
    approach_candidates = self.generate_cone_approach_candidates(
        ideal_approach, max_angle=30
    )
    
    # 마커 위치
    marker_pos = np.array([
        marker_pose_base.pose.position.x,
        marker_pose_base.pose.position.y,
        marker_pose_base.pose.position.z
    ])
    
    # IK 가능성으로 후보 필터링
    valid_approaches = self.filter_approaches_by_ik_feasibility(
        approach_candidates, marker_pos
    )
    
    return valid_approaches

def generate_cone_approach_candidates(self, ideal_approach, max_angle=30):
    """원추형 영역 내에서 접근 방향 후보 생성"""
    
    candidates = []
    angle_step = 10  # degrees (조정 가능)
    
    # 이상적 접근 방향 (원추 중심축)
    candidates.append(ideal_approach)
    
    for theta in range(angle_step, max_angle + 1, angle_step):  # 원추 각도
        for phi in range(0, 360, angle_step * 3):  # 회전 각도 (120도씩)
            
            # 구면 좌표계에서 직교 좌표계로 변환
            theta_rad = np.radians(theta)
            phi_rad = np.radians(phi)
            
            # 법선을 중심축으로 하는 원추 위의 점
            local_direction = np.array([
                np.sin(theta_rad) * np.cos(phi_rad),
                np.sin(theta_rad) * np.sin(phi_rad), 
                np.cos(theta_rad)
            ])
            
            # 로컬 좌표계를 글로벌 좌표계로 변환
            # 이상적 접근 방향을 z축으로 하는 변환 행렬 구성
            z_axis = ideal_approach / np.linalg.norm(ideal_approach)
            
            # 임의의 수직 벡터 생성
            if abs(z_axis[2]) < 0.9:  # z축과 평행하지 않은 경우
                x_axis = np.cross([0, 0, 1], z_axis)
            else:  # z축과 평행한 경우
                x_axis = np.cross([1, 0, 0], z_axis)
                
            x_axis = x_axis / np.linalg.norm(x_axis)
            y_axis = np.cross(z_axis, x_axis)
            
            # 회전 변환 행렬
            rotation_matrix = np.column_stack([x_axis, y_axis, z_axis])
            global_direction = rotation_matrix @ local_direction
            
            candidates.append(global_direction)
    
    return candidates

def filter_approaches_by_ik_feasibility(self, approach_candidates, marker_pos):
    """IK 해가 존재하는 접근 방향만 선별"""
    
    valid_approaches = []
    
    for approach_dir in approach_candidates:
        # 접근 및 누르기 위치 계산
        approach_pos = marker_pos + approach_dir * 0.05  # 5cm 떨어진 위치
        press_pos = marker_pos + approach_dir * 0.005    # 0.5cm 떨어진 위치
        
        # 기본 작업공간 확인
        if not (self.is_position_in_workspace(approach_pos) and 
                self.is_position_in_workspace(press_pos)):
            continue
            
        # 간단한 IK 가능성 사전 확인
        if self.quick_ik_feasibility_check(approach_pos):
            valid_approaches.append((approach_dir, approach_pos, press_pos))
    
    # 우선순위 정렬 (이상적 접근에 가까운 순)
    valid_approaches.sort(key=lambda x: self.calculate_approach_score(x[0], marker_pos))
    
    return valid_approaches

def quick_ik_feasibility_check(self, target_position):
    """빠른 IK 가능성 사전 확인"""
    x, y, z = target_position
    
    # 베이스에서의 거리 확인
    distance_2d = math.sqrt(x**2 + y**2)
    
    # 대략적인 최대 도달 거리 (링크 길이 합)
    max_reach = 0.1035 + 0.1275 + 0.092  # joint_3 + joint_4 + ee 길이
    
    # 높이 제한 확인 (베이스 기준)
    if z < 0.05 or z > 0.5:  # 5cm ~ 50cm 높이 범위
        return False
        
    # 2D 도달 가능성 확인
    if distance_2d > max_reach or distance_2d < 0.05:
        return False
        
    return True

def calculate_approach_score(self, approach_direction, marker_pos):
    """접근 방향 점수 계산 (낮을수록 우선순위 높음)"""
    # 기본 점수: 0 (이상적 접근)
    score = 0.0
    
    # 현재 로봇 자세와의 거리 (추후 구현)
    # score += self.calculate_joint_space_distance(approach_direction)
    
    # 접근 각도 편차 (법선에서 멀어질수록 페널티)
    angle_deviation = self.calculate_angle_deviation(approach_direction, marker_pos)
    score += angle_deviation * 0.1
    
    return score

def is_position_in_workspace(self, position):
    """위치가 로봇 작업공간 내에 있는지 확인"""
    x, y, z = position
    
    # 원통형 작업공간 가정
    radius = math.sqrt(x**2 + y**2)
    
    return (0.05 <= radius <= 0.35 and  # 5cm ~ 35cm 반경
            0.05 <= z <= 0.5)           # 5cm ~ 50cm 높이
```

### 2.4 에러 처리 강화
```python
def estimate_marker_pose_robust(self, marker_corners):
    """여러 solvePnP 알고리즘 시도로 신뢰성 향상"""
    algorithms = [
        cv2.SOLVEPNP_ITERATIVE,
        cv2.SOLVEPNP_SQPNP,
        cv2.SOLVEPNP_EPNP
    ]
    
    for algorithm in algorithms:
        try:
            success, rvec, tvec = cv2.solvePnP(
                self.aruco_3d_points, 
                marker_corners,
                self.camera_matrix, 
                self.dist_coeffs,
                flags=algorithm
            )
            
            if success and self.validate_pose_result(tvec):
                return True, rvec, tvec
                
        except Exception as e:
            self.get_logger().warn(f'solvePnP algorithm {algorithm} failed: {e}')
            continue
            
    return False, None, None
```

---

## 3. kinematics_solver.py 구현

### 3.1 URDF 파서
```python
class URDFKinematicsParser:
    """URDF에서 운동학 정보 추출"""
    
    def __init__(self, urdf_path=None):
        self.joint_info = {}
        self.link_info = {}
        self.parse_urdf(urdf_path)
    
    def parse_urdf(self, urdf_path):
        """URDF 파싱하여 조인트 정보 추출"""
        # ros2 param get /robot_state_publisher robot_description 활용
        # 또는 urdf_parser_py 사용
        
        # 추출할 정보:
        # - Joint origins (xyz, rpy)
        # - Joint limits
        # - Joint axes
        pass
    
    def get_dh_parameters(self):
        """DH 파라미터 계산"""
        # URDF joint origins을 DH 표준으로 변환
        return {
            'a': [0.0, 0.02, 0.1035, 0.1275],     # 링크 길이
            'd': [0.0814, 0.015, 0.0, 0.0],        # 링크 오프셋  
            'alpha': [0.0, -π/2, 0.0, 0.0],       # 링크 비틀림
            'theta_offset': [0.0, 0.0, 0.0, 0.0]  # 조인트 오프셋
        }
```

### 3.2 4-DOF IK Solver
```python
class DirectTransformIK:
    """직접 변환 행렬 기반 4-DOF IK 솔버"""
    
    def __init__(self, urdf_parser):
        self.urdf_parser = urdf_parser
        self.joint_chain = urdf_parser.get_joint_chain_info()
        
        # 벽면 접근 설정
        self.max_approach_angle = 30  # degrees
        
        # 조인트 제한
        self.joint_limits = {
            'min': [-π/2, -π/2, -π/2, -π/2],
            'max': [π/2, π/2, π/2, π/2]
        }
        
        # 작업공간 제한
        self.workspace_radius = 0.35  # meters
    
    def create_transform_matrix(self, xyz, rpy, axis, angle):
        """단일 조인트 변환 행렬 생성"""
        T = np.eye(4)
        
        # 1. Translation (xyz)
        T[:3, 3] = xyz
        
        # 2. Fixed rotation (rpy)
        if any(rpy):
            R_fixed = self.rpy_to_rotation_matrix(rpy)
            T[:3, :3] = R_fixed
        
        # 3. Joint rotation (axis * angle)
        if axis is not None:
            R_joint = self.axis_angle_to_rotation_matrix(axis, angle)
            T[:3, :3] = T[:3, :3] @ R_joint
        
        return T
    
    def rpy_to_rotation_matrix(self, rpy):
        """Roll-Pitch-Yaw를 회전 행렬로 변환"""
        r, p, y = rpy
        
        # Z-Y-X 순서 (ROS 표준)
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(r), -np.sin(r)],
                       [0, np.sin(r), np.cos(r)]])
        
        Ry = np.array([[np.cos(p), 0, np.sin(p)],
                       [0, 1, 0],
                       [-np.sin(p), 0, np.cos(p)]])
        
        Rz = np.array([[np.cos(y), -np.sin(y), 0],
                       [np.sin(y), np.cos(y), 0],
                       [0, 0, 1]])
        
        return Rz @ Ry @ Rx
    
    def axis_angle_to_rotation_matrix(self, axis, angle):
        """축-각도를 회전 행렬로 변환 (로드리게스 공식)"""
        axis = np.array(axis) / np.linalg.norm(axis)  # 정규화
        K = np.array([[0, -axis[2], axis[1]],
                      [axis[2], 0, -axis[0]],
                      [-axis[1], axis[0], 0]])
        
        R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
        return R
    
    def forward_kinematics(self, joint_angles):
        """순기구학: 조인트 각도 → 엔드이펙터 위치/자세"""
        T_cumulative = np.eye(4)
        
        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'ee_joint']
        
        for i, joint_name in enumerate(joint_names):
            joint_info = self.joint_chain[joint_name]
            
            if joint_name == 'ee_joint':
                # Fixed joint
                T_joint = self.create_transform_matrix(
                    joint_info['xyz'], joint_info['rpy'], None, 0
                )
            else:
                # Revolute joint
                T_joint = self.create_transform_matrix(
                    joint_info['xyz'], joint_info['rpy'], 
                    joint_info['axis'], joint_angles[i]
                )
            
            T_cumulative = T_cumulative @ T_joint
        
        position = T_cumulative[:3, 3]
        orientation_matrix = T_cumulative[:3, :3]
        return position, orientation_matrix
    
    def inverse_kinematics_geometric(self, target_position, current_joints=None):
        """4-DOF 기하학적 역기구학 (직접 변환 행렬 기반)"""
        x, y, z = target_position
        
        # Joint 1: 베이스 회전 (Z축)
        theta1 = math.atan2(y, x)
        
        # Joint 1 변환 후 2D 문제로 축소
        # Joint 1 변환 적용
        T1 = self.create_transform_matrix(
            self.joint_chain['joint_1']['xyz'],
            self.joint_chain['joint_1']['rpy'],
            self.joint_chain['joint_1']['axis'],
            theta1
        )
        
        # 목표점을 Joint 1 좌표계로 변환
        target_homogeneous = np.array([x, y, z, 1])
        target_in_j1 = np.linalg.inv(T1) @ target_homogeneous
        
        # 2D 평면에서의 거리 계산 (Joint 1 이후)
        x1, y1, z1 = target_in_j1[:3]
        
        # Joint 2 오프셋 고려
        j2_offset = self.joint_chain['joint_2']['xyz']
        x1_adj = x1 - j2_offset[0]  # -0.02 오프셋 보정
        z1_adj = z1 - j2_offset[2]  # 0.015 오프셋 보정
        
        # 링크 길이들
        L2 = self.joint_chain['joint_3']['xyz'][2]  # 0.1035
        L3 = self.joint_chain['joint_4']['xyz'][2]  # 0.1275
        L4 = self.joint_chain['ee_joint']['xyz'][2] # 0.092
        
        # 유효 목표 거리
        target_distance = math.sqrt(x1_adj**2 + z1_adj**2)
        
        # 도달 가능성 검사
        max_reach = L2 + L3 + L4
        min_reach = abs(L2 - L3 - L4)
        
        if target_distance > max_reach or target_distance < min_reach:
            raise ValueError(f"Target unreachable: dist={target_distance:.3f}m, "
                           f"range=[{min_reach:.3f}, {max_reach:.3f}]m")
        
        # 역방향으로 해 계산 (End Effector부터)
        # 목표에서 L4만큼 뒤로 이동한 지점이 Joint 4 위치
        wrist_distance = target_distance - L4
        
        # Joint 3 계산 (코사인 법칙)
        cos_theta3 = (wrist_distance**2 - L2**2 - L3**2) / (2 * L2 * L3)
        cos_theta3 = np.clip(cos_theta3, -1.0, 1.0)
        
        # 두 가지 해: elbow up/down
        theta3_solutions = [math.acos(cos_theta3), -math.acos(cos_theta3)]
        
        valid_solutions = []
        for theta3 in theta3_solutions:
            # Joint 2 계산
            alpha = math.atan2(z1_adj, x1_adj)
            beta = math.atan2(L3 * math.sin(theta3), L2 + L3 * math.cos(theta3))
            theta2 = alpha - beta
            
            # Joint 4 계산 (엔드이펙터 방향 제어)
            theta4 = self.calculate_wrist_angle_for_wall(theta2, theta3)
            
            solution = [theta1, theta2, theta3, theta4]
            
            # 해 검증
            if (self.check_joint_limits(solution) and 
                self.verify_solution_accuracy(solution, target_position)):
                valid_solutions.append(solution)
        
        if not valid_solutions:
            # 기하학적 해 실패 시 수치해법 시도
            return self.numerical_optimization(target_position, current_joints)
        
        # 현재 자세와 가장 가까운 해 선택
        if current_joints:
            return self.select_closest_solution(valid_solutions, current_joints)
        else:
            return valid_solutions[0]
    
    def numerical_optimization(self, target_position, initial_guess=None):
        """수치 최적화 기반 IK (기하학적 해 실패 시 백업)"""
        from scipy.optimize import minimize
        
        def objective_function(joint_angles):
            """목적 함수: 위치 오차 최소화"""
            try:
                current_pos, _ = self.forward_kinematics(joint_angles)
                return np.linalg.norm(current_pos - target_position)
            except:
                return 1000.0  # 큰 페널티
        
        # 초기 추정값
        if initial_guess is None:
            initial_guess = [0.0, 0.0, 0.0, 0.0]
        
        # 조인트 제한
        bounds = [(self.joint_limits['min'][i], self.joint_limits['max'][i]) 
                  for i in range(4)]
        
        # 최적화 실행
        result = minimize(
            objective_function,
            initial_guess,
            bounds=bounds,
            method='L-BFGS-B'
        )
        
        if result.success and result.fun < 0.01:  # 1cm 이내
            return result.x.tolist()
        else:
            raise ValueError(f"Numerical IK failed: error={result.fun:.6f}m")
    
    def calculate_wrist_angle_for_wall(self, theta2, theta3):
        """벽면 접근을 위한 손목 각도 계산"""
        # 엔드이펙터가 수평을 유지하도록 (간단한 경우)
        # 더 정교한 계산은 벽면 법선 방향을 고려해야 함
        return -(theta2 + theta3)
    
    def check_joint_limits(self, joint_angles):
        """조인트 제한 검사"""
        for i, angle in enumerate(joint_angles):
            if not (self.joint_limits['min'][i] <= angle <= self.joint_limits['max'][i]):
                return False
        return True
    
    def verify_solution_accuracy(self, joint_angles, target_position, tolerance=0.01):
        """해의 정확성 검증"""
        try:
            fk_position, _ = self.forward_kinematics(joint_angles)
            error = np.linalg.norm(fk_position - target_position)
            return error < tolerance
        except:
            return False
    
    def select_closest_solution(self, solutions, current_joints):
        """현재 자세와 가장 가까운 해 선택"""
        min_distance = float('inf')
        best_solution = solutions[0]
        
        for solution in solutions:
            distance = sum((a - b)**2 for a, b in zip(solution, current_joints))
            if distance < min_distance:
                min_distance = distance
                best_solution = solution
        
        return best_solution
```

### 3.3 수치 최적화 백업
```python
def numerical_optimization(self, target_position, approach_direction):
    """scipy.optimize를 사용한 수치 IK 해법"""
    from scipy.optimize import minimize
    
    def objective_function(joint_angles):
        """목적 함수: 위치 오차 + 접근 방향 오차"""
        current_pos, current_orient = self.forward_kinematics(joint_angles)
        
        pos_error = np.linalg.norm(current_pos - target_position)
        orient_error = self.calculate_orientation_error(current_orient, approach_direction)
        
        return pos_error + 0.1 * orient_error  # 가중치 적용
    
    # 초기 추정값 (현재 조인트 각도 또는 중간값)
    initial_guess = [0.0, 0.0, 0.0, 0.0]
    
    # 제약 조건
    bounds = [(self.joint_limits['min'][i], self.joint_limits['max'][i]) 
              for i in range(4)]
    
    # 최적화 실행
    result = minimize(
        objective_function, 
        initial_guess,
        bounds=bounds,
        method='L-BFGS-B'
    )
    
    if result.success and result.fun < 0.01:  # 1cm 이내 오차
        return result.x.tolist()
    else:
        raise ValueError("Numerical IK optimization failed")
```

---

## 4. 통합 워크플로우

### 4.1 전체 실행 순서
```python
def execute_button_click_sequence(self, target_marker_id):
    """전체 버튼 클릭 시퀀스 (현재 조인트 상태 활용)"""
    
    # 1. 관측 자세로 이동
    self.move_to_observation_pose()
    
    # 2. 마커 감지 대기 (current_joint_states는 자동 업데이트됨)
    marker_data = self.wait_for_marker_detection(target_marker_id)
    
    # 3. 카메라 좌표계에서 마커 3D 위치 추정
    marker_pose_camera = self.estimate_marker_pose(marker_data)
    
    # 4. TF2로 베이스 좌표계 변환
    marker_pose_ee, marker_pose_base = self.transform_marker_to_base(marker_pose_camera)
    
    # 5. 벽면 접근 자세 후보 생성
    approach_candidates = self.calculate_wall_approach_poses(marker_pose_base)
    
    # 6. 각 후보에 대해 IK 계산 (현재 조인트 상태를 초기값으로 활용)
    for approach_dir, approach_pos, press_pos in approach_candidates:
        try:
            joints_approach = self.ik_solver.inverse_kinematics_geometric(
                approach_pos, current_joints=self.current_joint_states
            )
            joints_press = self.ik_solver.inverse_kinematics_geometric(
                press_pos, current_joints=joints_approach  # 이전 결과를 다음 초기값으로
            )
            
            # 7. 버튼 누르기 동작 실행
            success = self.execute_press_sequence(joints_approach, joints_press)
            if success:
                return True
                
        except Exception as e:
            self.get_logger().warn(f'IK failed for candidate: {e}')
            continue
    
    # 8. 모든 후보 실패 시
    raise RuntimeError("No feasible approach found for wall marker")
```

### 4.2 테스트 및 검증
```python
# 단위 테스트 항목:
# 1. URDF 파싱 정확성
# 2. 순기구학 ↔ 역기구학 일관성  
# 3. TF2 변환 체인 검증
# 4. 벽면 접근 각도 계산
# 5. 조인트 제한 준수

# 통합 테스트 항목:
# 1. 가제보 시뮬레이션에서 전체 워크플로우
# 2. 다양한 마커 위치/각도에서 성공률
# 3. 에러 상황에서 복구 능력
```

---

## 5. 구현 우선순위

### Phase 1: 핵심 기능 구현
1. URDF 파서 구현
2. 기본 4-DOF IK 솔버 구현
3. button_click_node TF2 변환 로직 수정

### Phase 2: 벽면 특화 기능
1. 벽면 접근 자세 계산 로직
2. 수치 최적화 백업 시스템  
3. 에러 처리 및 복구 메커니즘

### Phase 3: 테스트 및 최적화
1. 단위 테스트 작성
2. 가제보 통합 테스트
3. 성능 최적화 및 파라미터 튜닝

이 설계 문서를 기반으로 코드 구현을 진행하면 체계적이고 안정적인 시스템을 구축할 수 있습니다.



ROS2 Jazzy + Gazebo Harmonic 환경에서 4-DOF 로봇팔의 벽면 ArUco 마커 버튼 클릭 시스템 리팩토링

설정 상수 정의
python# 카메라 파라미터 (실제 캘리브레이션 데이터)
CAMERA_MATRIX = np.array([
    [693.32, 0, 300.69],
    [0, 692.29, 281.98],
    [0, 0, 1]], dtype=np.float32)

DIST_COEFFS = np.array([
    -0.4149, 0.2780, 0.0004, -0.0002, -0.1941
], dtype=np.float32)

# 이미지 해상도
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

# 마커 설정
MARKER_SIZE = 0.035  # 3.5cm

# 접근 거리
APPROACH_DISTANCE = 0.05   # 5cm
PRESS_DISTANCE = 0.005     # 0.5cm

# 원추형 접근 설정
MAX_APPROACH_ANGLE = 30    # degrees
CONE_ANGLE_STEP = 10       # degrees
CONE_PHI_STEP = 30         # degrees

# 조인트 제한
JOINT_LIMITS_MIN = [-π/2, -π/2, -π/2, -π/2]
JOINT_LIMITS_MAX = [π/2, π/2, π/2, π/2]

# 작업공간 제한
WORKSPACE_RADIUS_MIN = 0.05  # 5cm
WORKSPACE_RADIUS_MAX = 0.35  # 35cm  
WORKSPACE_HEIGHT_MIN = 0.05  # 5cm
WORKSPACE_HEIGHT_MAX = 0.50  # 50cm

# 자세 정의
INITIAL_POSE = [0.0, 0.0, 0.0, 0.0]
OBSERVATION_POSE = [0.0, -0.7, 1.0, 0.6]
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4']

1. 시스템 아키텍처
좌표계 변환 체인
solvePnP → camera_link → ee_link → base_link → IK 배치계산 → 순차실행

2. button_click_node.py 리팩토링
2.1 현재 조인트 상태 관리
pythonclass ButtonClickNode(Node):
    def __init__(self):
        # 실시간 조인트 상태 추적
        self.joint_state_subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.current_joint_states = INITIAL_POSE.copy()
        
        # 카메라 파라미터 설정
        self.camera_matrix = CAMERA_MATRIX
        self.dist_coeffs = DIST_COEFFS
        self.image_width = IMAGE_WIDTH
        self.image_height = IMAGE_HEIGHT
        
    def joint_state_callback(self, msg):
        """ros2_control에서 현재 조인트 상태 수신"""
        joint_positions = []
        for joint_name in JOINT_NAMES:
            idx = msg.name.index(joint_name)
            joint_positions.append(msg.position[idx])
        self.current_joint_states = joint_positions
2.2 좌표계 변환 (2단계 TF2)
pythondef transform_marker_to_base(self, marker_pose_camera):
    """camera_link → ee_link → base_link"""
    try:
        # Step 1: camera_link → ee_link
        transform_cam_to_ee = self.tf_buffer.lookup_transform(
            'ee_link', 'camera_link', rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=1.0)
        )
        marker_pose_ee = do_transform_pose_stamped(marker_pose_camera, transform_cam_to_ee)
        
        # Step 2: ee_link → base_link
        transform_ee_to_base = self.tf_buffer.lookup_transform(
            'base_link', 'ee_link', rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=1.0)
        )
        marker_pose_base = do_transform_pose_stamped(marker_pose_ee, transform_ee_to_base)
        
        return marker_pose_base
        
    except Exception as e:
        self.get_logger().error(f'TF2 transform failed: {e}')
        return None
2.3 원추형 접근 후보 생성
pythondef generate_cone_approach_candidates(self, ideal_approach):
    """원추형 영역에서 접근 방향 후보 생성"""
    candidates = [ideal_approach]  # 중심축
    
    for theta in range(CONE_ANGLE_STEP, MAX_APPROACH_ANGLE + 1, CONE_ANGLE_STEP):
        for phi in range(0, 360, CONE_PHI_STEP):
            # 구면 좌표계 → 직교 좌표계
            theta_rad, phi_rad = np.radians(theta), np.radians(phi)
            local_dir = np.array([
                np.sin(theta_rad) * np.cos(phi_rad),
                np.sin(theta_rad) * np.sin(phi_rad), 
                np.cos(theta_rad)
            ])
            
            # 글로벌 좌표계로 변환
            z_axis = ideal_approach / np.linalg.norm(ideal_approach)
            x_axis = np.cross([0, 0, 1] if abs(z_axis[2]) < 0.9 else [1, 0, 0], z_axis)
            x_axis = x_axis / np.linalg.norm(x_axis)
            y_axis = np.cross(z_axis, x_axis)
            
            rotation_matrix = np.column_stack([x_axis, y_axis, z_axis])
            global_direction = rotation_matrix @ local_dir
            candidates.append(global_direction)
    
    return candidates

def filter_by_workspace(self, approach_candidates, marker_pos):
    """작업공간 내 후보만 선별"""
    valid_approaches = []
    
    for approach_dir in approach_candidates:
        approach_pos = marker_pos + approach_dir * APPROACH_DISTANCE
        press_pos = marker_pos + approach_dir * PRESS_DISTANCE
        
        if (self.is_in_workspace(approach_pos) and self.is_in_workspace(press_pos)):
            valid_approaches.append((approach_dir, approach_pos, press_pos))
    
    return valid_approaches

def is_in_workspace(self, position):
    """작업공간 검사"""
    x, y, z = position
    radius = math.sqrt(x**2 + y**2)
    return (WORKSPACE_RADIUS_MIN <= radius <= WORKSPACE_RADIUS_MAX and
            WORKSPACE_HEIGHT_MIN <= z <= WORKSPACE_HEIGHT_MAX)

3. kinematics_solver.py 구현
3.1 URDF 동적 파싱
pythonclass URDFKinematicsParser:
    def __init__(self):
        self.joint_chain = self.parse_urdf_from_ros_param()
    
    def parse_urdf_from_ros_param(self):
        """robot_state_publisher에서 URDF 읽기"""
        try:
            node = rclpy.create_node('temp_urdf_reader')
            client = node.create_client(GetParameters, '/robot_state_publisher/get_parameters')
            
            request = GetParameters.Request()
            request.names = ['robot_description']
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future)
            
            urdf_string = future.result().values[0].string_value
            node.destroy_node()
            
            return self.extract_joint_chain_from_urdf(urdf_string)
        except:
            return self.get_fallback_joint_chain()
    
    def get_fallback_joint_chain(self):
        """백업 조인트 데이터 (URDF 파싱 실패 시)"""
        return {
            'joint_1': {'xyz': [0, 0, 0.0814], 'rpy': [0, 0, 0], 'axis': [0, 0, 1]},
            'joint_2': {'xyz': [-0.02, 0, 0.015], 'rpy': [0, 0, 0], 'axis': [0, -1, 0]},
            'joint_3': {'xyz': [-0.0021, 0, 0.1035], 'rpy': [0, 0, 0], 'axis': [0, 1, 0]},
            'joint_4': {'xyz': [-0.002, 0, 0.1275], 'rpy': [0, 0, 0], 'axis': [0, -1, 0]},
            'ee_joint': {'xyz': [0.01, 0, 0.092], 'rpy': [0, 0, 0], 'axis': None}
        }


3.2 직접 변환 행렬 IK 솔버
pythonclass DirectTransformIK:
    def __init__(self):
        self.urdf_parser = URDFKinematicsParser()
        self.joint_chain = self.urdf_parser.joint_chain
    
    def forward_kinematics(self, joint_angles):
        """순기구학: URDF 변환 행렬 체인"""
        T = np.eye(4)
        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'ee_joint']
        
        for i, joint_name in enumerate(joint_names):
            joint_info = self.joint_chain[joint_name]
            angle = joint_angles[i] if joint_name != 'ee_joint' else 0
            
            T_joint = self.create_transform_matrix(
                joint_info['xyz'], joint_info['rpy'], joint_info['axis'], angle
            )
            T = T @ T_joint
        
        return T[:3, 3], T[:3, :3]
    
    def inverse_kinematics(self, target_position, current_joints):
        """IK 계산 (기하학적 + 수치해법 백업)"""
        try:
            return self.geometric_ik(target_position, current_joints)
        except:
            return self.numerical_ik(target_position, current_joints)
    
    def geometric_ik(self, target_position, current_joints):
        """4-DOF 기하학적 IK"""
        x, y, z = target_position
        
        # Joint 1: 베이스 회전
        theta1 = math.atan2(y, x)
        
        # 2D 문제로 축소
        r = math.sqrt(x**2 + y**2)
        
        # 링크 길이 (URDF에서 추출)
        L2 = self.joint_chain['joint_3']['xyz'][2]  # 0.1035
        L3 = self.joint_chain['joint_4']['xyz'][2]  # 0.1275  
        L4 = self.joint_chain['ee_joint']['xyz'][2] # 0.092
        
        # 도달성 검사 unreachable: {target_dist:.3f}m > {max_reach:.3f}m")
        
        # 기하학적 계산 (코사인 법칙 등)
        # ... 구체적 계산 로직
        
        return [theta1, theta2, theta3, theta4]
    
    def numerical_ik(self, target_position, current_joints):
        """수치 최적화 IK"""
        from scipy.optimize import minimize
        
        def objective(joint_angles):
            pos, _ = self.forward_kinematics(joint_angles)
            return np.linalg.norm(pos - target_position)
        
        result = minimize(
            objective, current_joints, 
            bounds=[(JOINT_LIMITS_MIN[i], JOINT_LIMITS_MAX[i]) for i in range(4)]
        )
        
        if result.success and result.fun < 0.01:
            return result.x.tolist()
        else:
            raise ValueError(f"Numerical IK failed: {result.fun:.6f}m")

4. 통합 워크플로우
pythondef execute_callback(self, goal_handle):
    """배치 IK 계산 방식"""
    target_id = goal_handle.request.button_id
    
    # 1. 관측 자세로 이동
    self.move_to_joint_angles(OBSERVATION_POSE)
    
    # 2. 마커 감지 및 변환
    marker_pose_base = self.detect_and_transform_marker(target_id)
    
    # 3. 원추형 접근 후보 생성 및 필터링
    approach_candidates = self.generate_and_filter_approaches(marker_pose_base)
    
    # 4. 배치 IK 계산 (current_joint_states 기준)
    for approach_dir, approach_pos, press_pos in approach_candidates:
        try:
            joints_approach = self.ik_solver.inverse_kinematics(
                approach_pos, self.current_joint_states
            )
            joints_press = self.ik_solver.inverse_kinematics(
                press_pos, self.current_joint_states
            )
            joints_retract = joints_approach  # 후퇴 = 접근
            
            # 5. 순차 실행
            success = self.execute_sequence(joints_approach, joints_press, joints_retract)
            if success:
                goal_handle.succeed()
                return ClickButton.Result(success=True)
                
        except Exception as e:
            self.get_logger().warn(f'Approach failed: {e}')
            continue
    
    goal_handle.abort()
    return ClickButton.Result(success=False, message="No valid approach found")

def execute_sequence(self, joints_approach, joints_press, joints_retract):
    """3자세 순차 실행"""
    return (self.move_to_joint_angles(joints_approach) and
            self.move_to_joint_angles(joints_press) and  
            self.move_to_joint_angles(joints_retract))

5. 구현 우선순위

kinematics_solver.py 기본 구조 - URDF 파싱, 순기구학, 간단한 IK
button_click_node.py TF2 변환 - 조인트 상태 구독, 좌표 변환
원추형 접근 로직 - 후보 생성, 필터링
통합 테스트 - Gazebo 시뮬레이션 검증

깔끔하게 정리했습니다. DH 파라미터 없이 URDF 직접 활용, 상수 상단 정의, 핵심 로직만 남겼습니다.
        target_dist = math.sqrt(r**2 + (z - 0.0814)**2)  # 베이스 높이 보정
        max_reach = L2 + L3 + L4
        
        if target_dist > max_reach:
            raise ValueError(f"Target unreachable: {target_dist:.3f}m > {max_reach:.3f}m")
        
        # 기하학적 계산 (코사인 법칙 등)
        # ... 구체적 계산 로직
        
        return [theta1, theta2, theta3, theta4]
    
    def numerical_ik(self, target_position, current_joints):
        """수치 최적화 IK"""
        from scipy.optimize import minimize
        
        def objective(joint_angles):
            pos, _ = self.forward_kinematics(joint_angles)
            return np.linalg.norm(pos - target_position)
        
        result = minimize(
            objective, current_joints, 
            bounds=[(JOINT_LIMITS_MIN[i], JOINT_LIMITS_MAX[i]) for i in range(4)]
        )
        
        if result.success and result.fun < 0.01:
            return result.x.tolist()
        else:
            raise ValueError(f"Numerical IK failed: {result.fun:.6f}m")

4. 통합 워크플로우
pythondef execute_callback(self, goal_handle):
    """배치 IK 계산 방식"""
    target_id = goal_handle.request.button_id
    
    # 1. 관측 자세로 이동
    self.move_to_joint_angles(OBSERVATION_POSE)
    
    # 2. 마커 감지 및 변환
    marker_pose_base = self.detect_and_transform_marker(target_id)
    
    # 3. 원추형 접근 후보 생성 및 필터링
    approach_candidates = self.generate_and_filter_approaches(marker_pose_base)
    
    # 4. 배치 IK 계산 (current_joint_states 기준)
    for approach_dir, approach_pos, press_pos in approach_candidates:
        try:
            joints_approach = self.ik_solver.inverse_kinematics(
                approach_pos, self.current_joint_states
            )
            joints_press = self.ik_solver.inverse_kinematics(
                press_pos, self.current_joint_states
            )
            joints_retract = joints_approach  # 후퇴 = 접근
            
            # 5. 순차 실행
            success = self.execute_sequence(joints_approach, joints_press, joints_retract)
            if success:
                goal_handle.succeed()
                return ClickButton.Result(success=True)
                
        except Exception as e:
            self.get_logger().warn(f'Approach failed: {e}')
            continue
    
    goal_handle.abort()
    return ClickButton.Result(success=False, message="No valid approach found")

def execute_sequence(self, joints_approach, joints_press, joints_retract):
    """3자세 순차 실행"""
    return (self.move_to_joint_angles(joints_approach) and
            self.move_to_joint_angles(joints_press) and  
            self.move_to_joint_angles(joints_retract))

5. 구현 우선순위

kinematics_solver.py 기본 구조 - URDF 파싱, 순기구학, 간단한 IK
button_click_node.py TF2 변환 - 조인트 상태 구독, 좌표 변환
원추형 접근 로직 - 후보 생성, 필터링
통합 테스트 - Gazebo 시뮬레이션 검증

깔끔하게 정리했습니다. DH 파라미터 없이 URDF 직접 활용, 상수 상단 정의, 핵심 로직만 남겼습니다.

💡 추가 제안사항
장기 개선 방향

GUI 도구: 마커 위치 시각화 및 캘리브레이션 도구
동적 파라미터: 런타임에 설정 변경 가능
성능 모니터링: 각 단계별 실행 시간 및 성공률 추적
안전 기능: 충돌 감지, 비상 정지 등

개발 도구 활용

디버깅: rqt_graph, ros2 topic echo 적극 활용
시각화: rviz2를 통한 좌표계 및 궤적 확인
프로파일링: Python cProfile로 성능 병목 지점 파악

이 계획을 따라 진행하시면, 현재의 불안정한 시스템을 견고하고 확장 가능한 아키텍처로 개선할 수 있습니다!
