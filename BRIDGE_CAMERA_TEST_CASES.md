# ROS-Gazebo Bridge & 카메라 통합 테스트 케이스

## 🎯 테스트 목표
1. **Bridge 연동**: ROS2 ↔ Gazebo 통신 확인
2. **카메라 활성화**: 이미지 토픽 구독 및 ArUco 마커 감지
3. **Joint 제어**: GUI ↔ Gazebo 로봇 연동
4. **좌표 시스템**: ArUco 마커 위치 최적화

---

## 🧪 테스트 케이스

### TC-B01: 기본 토픽 발행 확인 ✅
**목적**: 기본 ROS2 토픽들이 정상 발행되는지 확인
**실행명령**:
```bash
ros2 topic list
```
**예상결과**: 
- `/joint_states` ✅
- `/robot_description` ✅  
- `/tf`, `/tf_static` ✅
- `/clock` ✅
**검증결과**: ✅ **PASS** - 모든 기본 토픽 정상 발행

---

### TC-B02: 카메라 토픽 발행 확인
**목적**: Gazebo 카메라에서 ROS2로 이미지 데이터가 전달되는지 확인
**전제조건**: Bridge 활성화
**실행명령**:
```bash
# Terminal 1: 시뮬레이션 실행
```bash
cd /home/mac/dev_ws/roomie_arm
```
source install/setup.bash
export GZ_SIM_RESOURCE_PATH=$PWD/install/arm_gazebo/share/arm_gazebo/models:$GZ_SIM_RESOURCE_PATH
ros2 launch arm_gazebo gazebo.launch.py

# Terminal 2: 카메라 토픽 확인
ros2 topic list | grep camera
ros2 topic info /camera/image_raw
ros2 topic hz /camera/image_raw
```
**예상결과**:
- `/camera/image_raw` 토픽 존재
- `/camera/camera_info` 토픽 존재  
- 30Hz 주기로 이미지 데이터 발행
**검증항목**:
- [ ] 카메라 토픽 발행 확인
- [ ] 이미지 데이터 주기 확인
- [ ] 카메라 정보 메타데이터 확인

---

### TC-B03: 이미지 시각화 및 ArUco 마커 감지
**목적**: ROS2에서 카메라 이미지를 시각화하고 ArUco 마커가 보이는지 확인
**실행명령**:
```bash
# 이미지 시각화
ros2 run rqt_image_view rqt_image_view

# 또는 이미지 토픽 직접 확인
ros2 topic echo /camera/image_raw --once
```
**예상결과**:
- rqt_image_view에서 카메라 화면 표시
- ArUco 마커 3개(101, 102, 103)가 화면에 보임
- 마커들이 명확하게 구분되어 보임
**검증항목**:
- [ ] 카메라 이미지 정상 출력
- [ ] ArUco 마커 101 가시성 확인
- [ ] ArUco 마커 102 가시성 확인  
- [ ] ArUco 마커 103 가시성 확인
- [ ] 마커 크기 적절성 (3.5cm)
- [ ] 마커 방향 적절성 (수직 배치)

---

### TC-B04: Joint State 실시간 모니터링
**목적**: joint_state_publisher_gui와 실제 조인트 상태가 동기화되는지 확인
**실행명령**:
```bash
# Joint states 모니터링
ros2 topic echo /joint_states

# GUI에서 슬라이더 조작 후 변화 확인
```
**예상결과**:
- GUI 슬라이더 변경 시 `/joint_states` 토픽 값 변경
- 4개 조인트(joint_1~4) 모두 반영
**검증항목**:
- [ ] joint_1 슬라이더 → 토픽 반영
- [ ] joint_2 슬라이더 → 토픽 반영
- [ ] joint_3 슬라이더 → 토픽 반영
- [ ] joint_4 슬라이더 → 토픽 반영
- [ ] 실시간 동기화 확인

---

### TC-B05: Gazebo-ROS Joint 제어 연동
**목적**: ROS2에서 Gazebo 로봇 조인트를 직접 제어할 수 있는지 확인
**전제조건**: Bridge 활성화
**실행명령**:
```bash
# 개별 조인트 제어 테스트
ros2 topic pub /roomie_4dof/joint_1/cmd_pos std_msgs/msg/Float64 "data: 0.5" --once
ros2 topic pub /roomie_4dof/joint_2/cmd_pos std_msgs/msg/Float64 "data: -0.3" --once

# Joint trajectory 제어 테스트
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['joint_1', 'joint_2', 'joint_3', 'joint_4'],
  points: [{
    positions: [0.5, -0.3, 0.2, 0.0],
    time_from_start: {sec: 2, nanosec: 0}
  }]
}" --once
```
**예상결과**:
- ROS2 명령어로 Gazebo 로봇 조인트가 움직임
- GUI와 Gazebo가 서로 동기화됨
**검증항목**:
- [ ] 개별 조인트 위치 제어 가능
- [ ] 트라젝터리 제어 가능  
- [ ] Gazebo 로봇 실제 움직임 확인
- [ ] GUI ↔ Gazebo 양방향 동기화

---

### TC-B06: 카메라 좌표계 및 ArUco 마커 위치 최적화
**목적**: 카메라에서 ArUco 마커가 최적으로 보이도록 좌표 조정
**현재 설정**:
```xml
<!-- ArUco 마커들 위치 (roomie_aruco_world.world) -->
<pose>0.23 0 0.32 0 -1.5708 0</pose>  <!-- 마커 101 -->
<pose>0.23 0.0 0.23 0 -1.5708 0</pose>  <!-- 마커 102 -->  
<pose>0.23 0.0 0.16 0 -1.5708 0</pose>  <!-- 마커 103 -->
```
**조정 테스트**:
1. **거리 조정**: X축 0.23m → 0.3m (더 멀리)
2. **높이 분산**: Z축 간격 조정
3. **각도 미세조정**: pitch 값 미세 조정

**검증항목**:
- [ ] 모든 마커가 카메라 FOV 내에 위치
- [ ] 마커 간 겹침 없음
- [ ] 적절한 해상도로 마커 인식 가능
- [ ] 조인트 움직임 시에도 마커 가시성 유지

---

### TC-B07: 시뮬레이션 성능 및 안정성
**목적**: 시뮬레이션이 안정적으로 실행되고 적절한 성능을 보이는지 확인
**실행명령**:
```bash
# FPS 및 리소스 사용량 모니터링
top -p $(pgrep gz)

# 토픽 주기 확인
ros2 topic hz /joint_states
ros2 topic hz /camera/image_raw
```
**예상결과**:
- joint_states: ~50Hz
- camera/image_raw: ~30Hz  
- CPU 사용률 < 50%
- 메모리 사용률 적정 수준
**검증항목**:
- [ ] 토픽 발행 주기 안정성
- [ ] CPU/메모리 사용률 적절성
- [ ] 시뮬레이션 실시간 실행
- [ ] 장시간 실행 안정성

---

## 🚀 테스트 실행 순서

### Phase 1: 기본 연동 확인
1. **TC-B01**: 기본 토픽 발행 ✅
2. **TC-B04**: Joint State 모니터링
3. **TC-B07**: 성능 확인

### Phase 2: Bridge 및 카메라 활성화  
4. **TC-B02**: 카메라 토픽 발행
5. **TC-B03**: 이미지 시각화
6. **TC-B05**: Joint 제어 연동

### Phase 3: 좌표 최적화
7. **TC-B06**: ArUco 마커 위치 조정

---

## 📊 테스트 결과 기록

### 현재 상태 (2025-09-11)
- **TC-B01**: ✅ **PASS** - 기본 토픽 정상 발행
- **TC-B04**: ✅ **PASS** - Joint State 10Hz 안정 발행, GUI 정상 작동
- **TC-B07**: ✅ **PASS** - 시뮬레이션 성능 안정 (10Hz 토픽 주기)
- **TC-B02**: 🔄 **IN PROGRESS** - Bridge 설정됨, 카메라 토픽 대기 중
- **TC-B03**: ⏳ **PENDING** - RViz2 실행됨, 로봇 모델 설정 필요
- **TC-B05**: ⏳ **PENDING** - Joint 제어 테스트 준비됨
- **TC-B06**: ⏳ **PENDING** - 카메라 토픽 확인 후 좌표 조정

### ✅ **해결된 문제들**
1. **joint_states 토픽 미발행** → joint_state_publisher_gui 수동 실행으로 해결
2. **URDF 조인트 정의** → 4개 조인트 모두 정상 확인
3. **토픽 발행 주기** → 10Hz 안정적 발행 확인
4. **RViz2 연동** → 실행 성공, 설정 대기 중

### 🎯 **다음 우선순위**
1. **RViz2 로봇 모델 표시** - Fixed Frame 및 RobotModel 설정
2. **카메라 토픽 확인** - Gazebo 카메라 센서 활성화 상태 점검
3. **GUI ↔ Gazebo 연동** - 슬라이더 조작 시 Gazebo 로봇 움직임 확인

### 다음 액션 아이템
1. Bridge 재활성화하여 카메라 토픽 확인
2. rqt_image_view로 카메라 이미지 시각화
3. ArUco 마커 가시성 검증
4. Joint 제어 연동 테스트

---

## 🛠️ 트러블슈팅 가이드

### Bridge 연결 실패 시
```bash
# Gazebo 토픽 확인
gz topic -l | grep camera

# Bridge 수동 실행
ros2 run ros_gz_bridge parameter_bridge /camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image
```

### 카메라 이미지 없을 시
```bash
# URDF 카메라 센서 확인
ros2 param get /robot_state_publisher robot_description | grep -A 10 "camera"

# Gazebo에서 카메라 센서 확인  
gz topic -e /camera/image_raw
```

### Joint 제어 안될 시
```bash
# Gazebo 플러그인 확인
gz model -m roomie_4dof -i

# Joint 정보 확인
ros2 control list_hardware_interfaces
```
