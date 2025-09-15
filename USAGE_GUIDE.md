# Roomie 4DOF Robot 사용 가이드

## 🚀 실행 모드

### 1. URDF 검증 모드 (개발/디버깅)
URDF 파일 구조 검증 및 빠른 조인트 테스트용

```bash
# 환경 설정 (프로젝트 루트 디렉토리로 이동)
cd <your_workspace_path>/roomie-arm-ros2-gazebo
source install/setup.bash

# URDF 검증 모드 실행
ros2 launch arm_bringup urdf_validation.launch.py

# RViz로 시각화 (선택사항)
rviz2 -d $(ros2 pkg prefix arm_description)/share/arm_description/rviz/urdf_config.rviz
```

**특징:**
- ⚡ 빠른 시작 (Gazebo 없음)
- 🎛️ GUI 슬라이더로 조인트 제어
- 🔧 URDF 수정 후 즉시 테스트 가능
- 💻 낮은 리소스 사용

### 2. 시뮬레이션 제어 모드 (실제 사용)
실제 로봇과 동일한 환경에서 물리 시뮬레이션

```bash
# 환경 설정 (프로젝트 루트 디렉토리로 이동)
cd <your_workspace_path>/roomie-arm-ros2-gazebo
source install/setup.bash

# 시뮬레이션 제어 모드 실행
ros2 launch arm_bringup simulation_control.launch.py

# 또는 GUI 없이 실행
ros2 launch arm_bringup simulation_control.launch.py gui:=false
```

**특징:**
- 🎮 실제 로봇과 동일한 제어 방식
- ⚖️ 물리 법칙 적용 (관성, 마찰, 중력)
- 📊 RQT Joint Trajectory Controller 사용
- 🔗 실제 배포 코드와 동일한 인터페이스

## 🎯 개발 워크플로우

### URDF 개발 시:
1. **Mode 1**로 빠른 검증 → URDF 수정 → 반복
2. 완성 후 **Mode 2**로 물리 테스트

### Controller 개발 시:
- **Mode 2**만 사용하여 실제 환경에서 테스트

## 📋 수동 제어 명령어

시뮬레이션 모드에서 터미널로 직접 제어:

```bash
# 개별 조인트 제어
ros2 topic pub /joint_trajectory_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, 0.3, -0.2, 0.1]"

# 홈 포지션
ros2 topic pub /joint_trajectory_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0]"

# 조인트 상태 확인
ros2 topic echo /joint_states
```

## 🔧 문제 해결

### URDF 메시 파일 오류:
```bash
colcon build --packages-select arm_description
source install/setup.bash
```

### Gazebo 시계 동기화 경고:
- 정상적인 동작이며 무시해도 됨
- GUI 없이 실행하려면: `gui:=false` 옵션 사용
