# RoomieArm Core Package - 테스트 및 디버깅 가이드

## 📋 개요
RoomieArm의 핵심 기능을 담당하는 패키지입니다. ArUco 마커 감지, 역기구학 계산, 버튼 클릭 자동화를 수행합니다.

## 🏗️ 아키텍처 분석

### 현재 구조
```
roomiearm_core/
├── vision_node.py          # ArUco 마커 감지 및 좌표 발행
├── button_click_node.py    # 버튼 클릭 작업 수행 (DirectTransformIK 사용)
└── kinematics_solver.py    # 두 개의 클래스:
    ├── DirectTransformIK   # ✅ 실제 사용됨 (핵심 클래스)
    └── KinematicsSolver    # ❌ 사용되지 않음 (불필요한 래퍼)
```

### ⚠️ 구조적 문제점
1. **KinematicsSolver 클래스 미사용**: button_click_node가 DirectTransformIK를 직접 사용
2. **중복 구조**: KinematicsSolver가 DirectTransformIK의 단순 래퍼 역할
3. **일관성 부족**: 어떤 클래스를 표준으로 사용할지 불명확

## 🔄 데이터 흐름

### 1. Vision Pipeline
```
Camera → vision_node → MarkerArray → button_click_node
```

### 2. Button Click Pipeline
```
ClickButton Action →
  1. 관측 자세 이동
  2. 마커 감지 대기
  3. solvePnP로 3D 자세 추정
  4. camera_link → ee_link → base_link TF 변환
  5. 원추형 접근 후보 생성
  6. DirectTransformIK로 역기구학 계산
  7. 접근 → 누르기 → 후퇴 시퀀스 실행
```

## 🧪 상세 테스트 케이스

### 1. Vision Node 테스트

#### 1.1 기본 동작 테스트
```bash
# 터미널 1: 노드 실행
ros2 run roomiearm_core vision_node

# 터미널 2: 토픽 확인
ros2 topic list | grep marker
ros2 topic echo /marker_array
```

**예상 결과**: ArUco 마커가 감지되면 MarkerArray 메시지 발행

#### 1.2 마커 감지 성능 테스트
```bash
# 마커 감지 빈도 확인
ros2 topic hz /marker_array

# 마커 감지 정확도 확인 (5초간 데이터 수집)
ros2 topic echo /marker_array --qos-reliability reliable --qos-durability volatile | head -50
```

### 2. Kinematics Solver 단위 테스트

#### 2.1 Forward Kinematics 테스트
```python
# 테스트 스크립트: test_kinematics.py
import sys
import os
sys.path.append('/home/mac/dev_ws/roomiearm-ros2-gazebo/roomiearm_core')

from roomiearm_core.kinematics_solver import DirectTransformIK

def test_forward_kinematics():
    """정기구학 테스트"""
    ik_solver = DirectTransformIK()

    # 테스트 케이스들
    test_cases = [
        [0.0, 0.0, 0.0, 0.0],      # 초기 자세
        [0.0, -0.7, 1.0, 0.6],     # 관측 자세
        [0.5, 0.3, -0.2, 0.1],     # 임의 자세
    ]

    for i, joints in enumerate(test_cases):
        try:
            pos, rot = ik_solver.forward_kinematics(joints)
            print(f"Test {i+1}: joints={joints}")
            print(f"  Position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")
            print(f"  Valid workspace: {ik_solver.is_in_workspace(pos)}")
            print()
        except Exception as e:
            print(f"Test {i+1} FAILED: {e}")

if __name__ == "__main__":
    test_forward_kinematics()
```

#### 2.2 Inverse Kinematics 테스트
```python
def test_inverse_kinematics():
    """역기구학 테스트"""
    ik_solver = DirectTransformIK()

    # 작업공간 내 목표 위치들
    test_positions = [
        [0.15, 0.0, 0.25],    # 정면 중간 높이
        [0.0, 0.15, 0.25],    # 좌측 중간 높이
        [0.10, 0.10, 0.30],   # 대각선 위치
        [0.20, 0.0, 0.15],    # 정면 낮은 위치
    ]

    current_joints = [0.0, 0.0, 0.0, 0.0]

    for i, target_pos in enumerate(test_positions):
        try:
            # 역기구학 계산
            joint_solution = ik_solver.inverse_kinematics(target_pos, current_joints)

            # 검증: 정기구학으로 역계산
            verify_pos, _ = ik_solver.forward_kinematics(joint_solution)
            error = [(target_pos[j] - verify_pos[j])**2 for j in range(3)]
            error_magnitude = sum(error)**0.5

            print(f"IK Test {i+1}: target={target_pos}")
            print(f"  Solution: {[f'{j:.3f}' for j in joint_solution]}")
            print(f"  Verification: {[f'{p:.3f}' for p in verify_pos]}")
            print(f"  Error: {error_magnitude:.6f}m")
            print(f"  Status: {'✅ PASS' if error_magnitude < 0.01 else '❌ FAIL'}")
            print()

        except Exception as e:
            print(f"IK Test {i+1} FAILED: {e}")
```

### 3. Button Click Node 통합 테스트

#### 3.1 기본 액션 테스트
```bash
# 터미널 1: button_click_node 실행
ros2 run roomiearm_core button_click_node

# 터미널 2: 액션 서버 확인
ros2 action list | grep click_button

# 터미널 3: 테스트 액션 호출
ros2 action send_goal /click_button roomiearm_msgs/action/ClickButton "{robot_id: 0, button_id: 1}"
```

#### 3.2 단계별 디버깅 테스트
```bash
# 각 단계별 토픽 모니터링
ros2 topic echo /marker_array &
ros2 topic echo /joint_states &
ros2 topic echo /joint_trajectory_controller/follow_joint_trajectory/goal &

# 액션 실행
ros2 action send_goal --feedback /click_button roomiearm_msgs/action/ClickButton "{robot_id: 0, button_id: 2}"
```

## 🐛 디버깅 로그 설정

### 1. 로깅 레벨 설정
```bash
# INFO 레벨로 실행
ros2 run roomiearm_core vision_node --ros-args --log-level INFO

# DEBUG 레벨로 실행 (상세 로그)
ros2 run roomiearm_core button_click_node --ros-args --log-level DEBUG
```

### 2. 코드 내 디버깅 로그 추가

#### vision_node.py 디버깅 포인트
```python
# Line 40 근처에 추가
self.get_logger().debug(f"Image processing: {h}x{w}, ArUco dict type: {type(self.aruco_dict)}")

# Line 53 근처에 추가
if ids is not None:
    self.get_logger().info(f"✅ Detected {len(ids)} markers: {ids.flatten().tolist()}")
    for i, marker_id in enumerate(ids):
        center_x = np.mean([c[0] for c in corners[i][0]])
        center_y = np.mean([c[1] for c in corners[i][0]])
        self.get_logger().debug(f"Marker {marker_id[0]}: center=({center_x:.1f}, {center_y:.1f})")
else:
    self.get_logger().warn("❌ No ArUco markers detected in frame")
```

#### button_click_node.py 디버깅 포인트
```python
# Line 335 근처 (마커 감지 후)
self.get_logger().info(f"🎯 Target marker {target_id} detected at base_link coordinates:")
self.get_logger().info(f"   Position: ({marker_pose_base.pose.position.x:.3f}, "
                      f"{marker_pose_base.pose.position.y:.3f}, "
                      f"{marker_pose_base.pose.position.z:.3f})")

# Line 355 근처 (각 접근 시도 전)
self.get_logger().info(f"🔄 Attempt {i+1}/{len(approach_candidates)}: "
                      f"approach_pos={approach_pos}, press_pos={press_pos}")

# Line 361 근처 (IK 계산 후)
self.get_logger().debug(f"   IK Solutions: approach={[f'{j:.3f}' for j in joints_approach]}, "
                       f"press={[f'{j:.3f}' for j in joints_press]}")

# Line 379 근처 (접근 실패 시)
self.get_logger().warn(f"❌ Approach {i+1} failed: {str(e)[:100]}...")
```

### 3. TF2 변환 디버깅
```python
# button_click_node.py의 transform_marker_to_base 함수에 추가
def transform_marker_to_base(self, marker_pose_camera):
    """camera_link → ee_link → base_link with debugging"""
    try:
        self.get_logger().debug(f"🔄 TF Transform: camera_link → ee_link")
        transform_cam_to_ee = self.tf_buffer.lookup_transform(
            'ee_link', 'camera_link', rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=1.0)
        )
        marker_pose_ee = do_transform_pose_stamped(marker_pose_camera, transform_cam_to_ee)
        self.get_logger().debug(f"   EE frame: ({marker_pose_ee.pose.position.x:.3f}, "
                               f"{marker_pose_ee.pose.position.y:.3f}, "
                               f"{marker_pose_ee.pose.position.z:.3f})")

        self.get_logger().debug(f"🔄 TF Transform: ee_link → base_link")
        transform_ee_to_base = self.tf_buffer.lookup_transform(
            'base_link', 'ee_link', rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=1.0)
        )
        marker_pose_base = do_transform_pose_stamped(marker_pose_ee, transform_ee_to_base)
        self.get_logger().debug(f"   Base frame: ({marker_pose_base.pose.position.x:.3f}, "
                               f"{marker_pose_base.pose.position.y:.3f}, "
                               f"{marker_pose_base.pose.position.z:.3f})")

        return marker_pose_base

    except Exception as e:
        self.get_logger().error(f"❌ TF2 transform failed: {e}")
        return None
```

## 🚨 문제 해결 체크리스트

### Vision Node 문제
- [ ] 카메라 토픽 `/camera/image_raw` 발행 여부 확인
- [ ] ArUco 마커가 카메라 시야에 있는지 확인
- [ ] 마커 크기 `MARKER_SIZE = 0.035`가 실제 마커와 일치하는지 확인
- [ ] 카메라 캘리브레이션 매개변수 `CAMERA_MATRIX`, `DIST_COEFFS` 검증

### Button Click Node 문제
- [ ] robot_state_publisher가 실행 중인지 확인
- [ ] TF2 트리 완전성 확인: `ros2 run tf2_tools view_frames`
- [ ] joint_trajectory_controller 활성화 상태 확인
- [ ] 작업공간 제한 매개변수들이 적절한지 검증

### Kinematics 문제
- [ ] URDF 파싱이 성공하는지 확인
- [ ] Joint limits가 실제 하드웨어와 일치하는지 확인
- [ ] Geometric IK vs Numerical IK 성능 비교

## 🔧 권장 구조 개선 사항

### Option 1: DirectTransformIK 표준화
```python
# KinematicsSolver 클래스 제거, DirectTransformIK만 사용
# 모든 곳에서 일관되게 DirectTransformIK 사용
```

### Option 2: KinematicsSolver 통일
```python
# button_click_node에서 KinematicsSolver 사용으로 변경
# from .kinematics_solver import KinematicsSolver
# self.ik_solver = KinematicsSolver()
```

현재 상황에서는 **Option 1 (DirectTransformIK 표준화)**을 권장합니다.