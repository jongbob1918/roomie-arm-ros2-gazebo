# RoomieArm 버튼 클릭 시스템 테스트 케이스

## 📋 테스트 환경 및 전제조건

### 시스템 구성
- **ROS2**: Jazzy
- **Gazebo**: Harmonic
- **로봇팔**: 4-DOF 평면 매니퓰레이터
- **카메라**: link_4에 장착된 RGB 카메라
- **마커**: ArUco DICT_4X4_250, 3.5cm 크기

### 의존성 패키지
```bash
# 필수 패키지 확인
ros2 pkg list | grep -E "(roomiearm|ros_gz|controller_manager)"
```

### 핵심 노드 및 토픽
- **노드**: `vision_node`, `button_click_node`, `robot_state_publisher`
- **액션**: `/click_button` (ClickButton.action)
- **토픽**: `/joint_states`, `/marker_array`, `/camera/image_raw`
- **컨트롤러**: `joint_trajectory_controller`, `joint_state_broadcaster`

---

## 🧪 테스트 시나리오

### **TC-001: 시스템 초기화 및 노드 기동**

#### TC-001-1: Gazebo 시뮬레이션 시작
```bash
# Terminal 1: Gazebo + ROS2 Control 시작
ros2 launch roomiearm_gazebo gazebo.launch.py gui:=true

# 예상 결과
✅ Gazebo GUI 실행
✅ roomiearm 로봇 로드 (base_link 기준 원점)
✅ ArUco 마커들이 벽면에 배치됨
✅ /joint_states 토픽 발행 시작
✅ joint_trajectory_controller 활성화
```

#### TC-001-2: Vision Node 시작
```bash
# Terminal 2: 비전 노드 시작
source install/setup.bash
ros2 run roomiearm_core vision_node

# 검증 명령어
ros2 topic echo /marker_array --once
ros2 topic hz /marker_array

# 예상 결과
✅ 카메라 이미지 수신 확인 (640x480)
✅ ArUco 마커 감지 및 /marker_array 발행
✅ 마커 ID와 corners 좌표 정상 출력
```

#### TC-001-3: Button Click Node 시작
```bash
# Terminal 3: 버튼 클릭 노드 시작
source install/setup.bash
ros2 run roomiearm_core button_click_node

# 예상 결과
✅ "Button Click Node initialized with new architecture" 로그
✅ 조인트 상태 구독 시작
✅ TF2 리스너 활성화
✅ /click_button 액션 서버 대기상태
```

---

### **TC-002: 키네마틱스 및 작업공간 검증**

#### TC-002-1: URDF 파싱 테스트
```bash
# Python 테스트 스크립트
python3 -c "
from roomiearm_core.kinematics_solver import URDFKinematicsParser
parser = URDFKinematicsParser()
print('Joint Chain:', parser.joint_chain)
"

# 예상 결과
✅ joint_1~4, ee_joint 정보 출력
✅ xyz, rpy, axis 값들이 URDF와 일치
✅ fallback 없이 정상 파싱
```

#### TC-002-2: 순기구학 검증
```bash
# Python 테스트
python3 -c "
from roomiearm_core.kinematics_solver import DirectTransformIK
ik = DirectTransformIK()
pos, rot = ik.forward_kinematics([0, 0, 0, 0])
print(f'Initial pose: {pos}')
pos, rot = ik.forward_kinematics([0, -0.7, 1.0, 0.6])
print(f'Observation pose: {pos}')
"

# 예상 결과
✅ Initial pose: [0.01, 0, ~0.39] (Z축 누적)
✅ Observation pose: 작업공간 내부 위치
✅ 회전행렬 정상 계산
```

#### TC-002-3: 작업공간 경계 테스트
```bash
# 경계값 테스트
python3 -c "
from roomiearm_core.kinematics_solver import DirectTransformIK
ik = DirectTransformIK()
test_points = [
    [0.05, 0, 0.1],      # 최소 반지름
    [0.34, 0, 0.2],      # 최대 반지름
    [0.2, 0, 0.05],      # 최소 높이
    [0.2, 0, 0.50],      # 최대 높이
    [0.40, 0, 0.3],      # 도달 불가
]
for pt in test_points:
    result = ik.is_in_workspace(pt)
    print(f'{pt} -> {result}')
"

# 예상 결과
✅ [0.05, 0, 0.1] -> True
✅ [0.34, 0, 0.2] -> True
✅ [0.2, 0, 0.05] -> True
✅ [0.2, 0, 0.50] -> True
✅ [0.40, 0, 0.3] -> False
```

---

### **TC-003: 마커 감지 및 좌표 변환**

#### TC-003-1: 관측 자세에서 마커 가시성
```bash
# GUI 컨트롤러로 관측 자세 이동
ros2 run roomiearm_bringup robot_gui_controller

# 관측 자세 [0, -0.7, 1.0, 0.6]로 이동 후
ros2 topic echo /marker_array --once

# 예상 결과
✅ 최소 1개 이상의 마커 감지
✅ marker_id와 corners 정상 출력
✅ 정규화된 좌표 (0~1 범위)
```

#### TC-003-2: solvePnP 3D 자세 추정
```bash
# 마커별 3D 위치 확인
ros2 topic echo /marker_array | grep -A 10 "marker_id"

# TF2 변환 체인 확인
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link camera_link

# 예상 결과
✅ camera_link -> ee_link 변환 존재
✅ ee_link -> base_link 변환 존재
✅ 마커 3D 좌표가 물리적으로 타당한 범위
```

#### TC-003-3: TF2 2단계 변환 검증
```bash
# 수동 TF2 변환 테스트
python3 -c "
import rclpy
from roomiearm_core.button_click_node import ButtonClickNode
rclpy.init()
node = ButtonClickNode()
# node.transform_marker_to_base() 테스트...
"

# 예상 결과
✅ camera_link -> ee_link 변환 성공
✅ ee_link -> base_link 변환 성공
✅ 최종 base_link 좌표 작업공간 내부
```

---

### **TC-004: 접근 경로 생성 및 IK 계산**

#### TC-004-1: 원추형 접근 후보 생성
```bash
# 접근 후보 개수 확인
python3 -c "
import numpy as np
from roomiearm_core.button_click_node import ButtonClickNode
node = ButtonClickNode()
ideal = np.array([0, 0, 1])  # Z축 방향
candidates = node.generate_cone_approach_candidates(ideal)
print(f'Generated {len(candidates)} approach candidates')
"

# 예상 결과
✅ 총 13개 후보 (중심 1개 + 12개 원추)
✅ 30도 원추각 내부 분포
✅ 정규화된 방향 벡터
```

#### TC-004-2: 작업공간 필터링
```bash
# 필터링 후 유효 후보 수
python3 -c "
import numpy as np
from geometry_msgs.msg import PoseStamped
from roomiearm_core.button_click_node import ButtonClickNode

# 테스트용 마커 자세 (작업공간 내부)
marker_pose = PoseStamped()
marker_pose.pose.position.x = 0.2
marker_pose.pose.position.y = 0.1
marker_pose.pose.position.z = 0.3
marker_pose.pose.orientation.w = 1.0

node = ButtonClickNode()
valid_approaches = node.generate_and_filter_approaches(marker_pose)
print(f'Valid approaches: {len(valid_approaches)}')
"

# 예상 결과
✅ 1개 이상의 유효한 접근 후보
✅ approach_pos와 press_pos 모두 작업공간 내부
✅ APPROACH_DISTANCE(5cm), PRESS_DISTANCE(0.5cm) 적용
```

#### TC-004-3: IK 솔버 안정성
```bash
# 다양한 목표점에 대한 IK 성공률
python3 -c "
from roomiearm_core.kinematics_solver import DirectTransformIK
import numpy as np

ik = DirectTransformIK()
test_points = [
    [0.15, 0.1, 0.2],
    [0.2, -0.15, 0.25],
    [0.25, 0.2, 0.3],
    [0.3, -0.1, 0.35]
]
success_count = 0
for pt in test_points:
    try:
        joints = ik.inverse_kinematics(pt, [0,0,0,0])
        success_count += 1
        print(f'{pt} -> SUCCESS: {joints}')
    except Exception as e:
        print(f'{pt} -> FAILED: {e}')
print(f'Success rate: {success_count}/{len(test_points)}')
"

# 예상 결과
✅ 80% 이상 성공률
✅ 조인트 제한 준수
✅ 도달 불가능한 점에 대해서는 명확한 에러
```

---

### **TC-005: 전체 통합 테스트**

#### TC-005-1: 기본 버튼 클릭 시퀀스
```bash
# 액션 클라이언트로 버튼 클릭 요청
ros2 action send_goal /click_button roomiearm_msgs/action/ClickButton "{robot_id: 0, button_id: 101}"

# 로그 모니터링
ros2 node info button_click_node
```

**예상 시퀀스:**
1. ✅ "Moving to observation pose..."
2. ✅ 조인트가 [0, -0.7, 1.0, 0.6]으로 이동
3. ✅ "Detecting and transforming marker..."
4. ✅ 마커 ID 101 감지 및 base_link 좌표 출력
5. ✅ "Generating approach candidates..."
6. ✅ N개 유효 접근 후보 생성
7. ✅ "Solving IK for approach candidates..."
8. ✅ 첫 번째 후보로 IK 성공
9. ✅ "Executing button press sequence..."
10. ✅ 접근 -> 누르기 -> 후퇴 동작 완료
11. ✅ "Returning to initial pose..."
12. ✅ "Button Click for ID 101 SUCCEEDED"

#### TC-005-2: 다중 마커 테스트
```bash
# 여러 마커에 순차 클릭
for id in 101 102 103; do
    echo "Testing marker $id"
    ros2 action send_goal /click_button roomiearm_msgs/action/ClickButton "{robot_id: 0, button_id: $id}" --feedback
    sleep 2
done

# 예상 결과
✅ 각 마커별 성공적인 클릭
✅ 서로 다른 접근 경로 사용
✅ 충돌 없이 안전한 동작
```

#### TC-005-3: 에러 상황 처리
```bash
# 존재하지 않는 마커 ID
ros2 action send_goal /click_button roomiearm_msgs/action/ClickButton "{robot_id: 0, button_id: 999}"

# 예상 결과
✅ "Target marker ID 999 not found" 에러
✅ 액션 abort 상태로 종료
✅ 시스템 안정성 유지

# 카메라 가려진 상태
# (수동으로 로봇을 다른 자세로 이동 후)
ros2 action send_goal /click_button roomiearm_msgs/action/ClickButton "{robot_id: 0, button_id: 101}"

# 예상 결과
✅ "Marker detection timeout" 또는 "No valid approach found"
✅ 안전한 에러 복구
✅ 초기 자세로 복귀 시도
```

---

### **TC-006: 성능 및 안전성 테스트**

#### TC-006-1: 응답 시간 측정
```bash
# 벤치마킹 스크립트
time ros2 action send_goal /click_button roomiearm_msgs/action/ClickButton "{robot_id: 0, button_id: 101}" --feedback

# 목표 성능
✅ 전체 시퀀스 완료: < 30초
✅ 마커 감지: < 3초
✅ IK 계산: < 1초
✅ 동작 실행: < 20초
```

#### TC-006-2: 조인트 제한 준수
```bash
# 조인트 모니터링
ros2 topic echo /joint_states | grep -A 4 "position"

# 클릭 동작 중 조인트 각도 확인
# 예상 결과
✅ 모든 조인트: -π/2 ~ π/2 범위 내
✅ 급격한 속도 변화 없음
✅ 부드러운 궤적 생성
```

#### TC-006-3: 메모리 및 CPU 사용량
```bash
# 리소스 모니터링
top -p $(pgrep -f button_click_node)
top -p $(pgrep -f vision_node)

# 목표 성능
✅ 메모리: < 100MB per node
✅ CPU: < 50% during operation
✅ 메모리 누수 없음 (장시간 실행)
```

---

### **TC-007: 로버스트니스 테스트**

#### TC-007-1: 연속 실행 안정성
```bash
# 100회 연속 클릭 테스트
for i in {1..100}; do
    echo "Test iteration: $i"
    ros2 action send_goal /click_button roomiearm_msgs/action/ClickButton "{robot_id: 0, button_id: 101}" --feedback
    if [ $? -ne 0 ]; then
        echo "Failed at iteration $i"
        break
    fi
    sleep 1
done

# 예상 결과
✅ 95% 이상 성공률
✅ 시스템 크래시 없음
✅ 메모리 누수 없음
```

#### TC-007-2: 노드 재시작 복구
```bash
# 비전 노드 강제 종료 후 재시작
pkill -f vision_node
sleep 2
ros2 run roomiearm_core vision_node &

# 버튼 클릭 테스트
ros2 action send_goal /click_button roomiearm_msgs/action/ClickButton "{robot_id: 0, button_id: 101}"

# 예상 결과
✅ 자동으로 마커 감지 재개
✅ 정상적인 버튼 클릭 동작
✅ 시스템 무결성 유지
```

---

## 🔧 디버깅 도구 및 명령어

### 실시간 모니터링
```bash
# 토픽 상태 확인
ros2 topic list
ros2 topic hz /joint_states /marker_array

# TF 트리 시각화
ros2 run tf2_tools view_frames

# 액션 상태 확인
ros2 action list
ros2 action info /click_button

# 컨트롤러 상태
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

### 로그 분석
```bash
# 노드별 로그 확인
ros2 node info button_click_node
ros2 node info vision_node

# 에러 로그 필터링
ros2 log set_logger_level button_click_node DEBUG
```

### 수동 테스트
```bash
# 개별 조인트 제어
ros2 run roomiearm_bringup robot_gui_controller

# 마커 감지 상태 확인
ros2 topic echo /marker_array --once

# TF2 변환 확인
ros2 run tf2_ros tf2_echo base_link camera_link
```

---

## 📊 합격 기준

### 필수 통과 테스트
- [ ] TC-001: 시스템 초기화 (모든 하위 테스트)
- [ ] TC-003-1: 관측 자세에서 마커 감지
- [ ] TC-005-1: 기본 버튼 클릭 시퀀스 성공
- [ ] TC-006-2: 조인트 제한 준수

### 권장 통과 테스트
- [ ] TC-002: 키네마틱스 검증 (80% 이상)
- [ ] TC-005-2: 다중 마커 테스트 (3개 중 2개 이상 성공)
- [ ] TC-006-1: 성능 기준 달성
- [ ] TC-007-1: 연속 실행 (95% 이상 성공률)

### 추가 검증 항목
- [ ] 카메라 캘리브레이션 데이터 정확성
- [ ] ArUco 마커 크기 및 배치 적절성
- [ ] Gazebo 물리 엔진 설정 최적화
- [ ] 실제 하드웨어 이식 가능성

---

## 🚨 알려진 제한사항

1. **카메라 FOV**: 관측 자세에서 모든 마커가 시야에 들어와야 함
2. **조명 조건**: Gazebo 환경에서만 테스트됨 (실제 조명 변화 미검증)
3. **마커 크기**: 3.5cm 고정 (다양한 크기 미지원)
4. **충돌 감지**: 현재 구현에는 충돌 회피 로직 없음
5. **실시간성**: ROS2 Action 기반으로 실시간 보장 불가

---

## 📝 테스트 보고서 템플릿

```markdown
# 테스트 실행 결과

**실행 일시**: YYYY-MM-DD HH:MM:SS
**테스터**: [이름]
**환경**: [OS, ROS2 버전, Gazebo 버전]

## 테스트 결과 요약
- 전체: X/Y 통과
- 필수: X/Y 통과
- 권장: X/Y 통과

## 실패한 테스트
- TC-XXX-X: [실패 이유 및 로그]

## 발견된 이슈
1. [이슈 설명]
2. [개선 제안]

## 전체 평가
- [ ] 시스템 배포 준비 완료
- [ ] 추가 개발 필요
- [ ] 심각한 결함으로 재설계 필요
```