# Gazebo 시뮬레이션 환경 설정 가이드

## 🎯 ArUco 마커 위치 및 크기 조절

### 파일 위치
```
/home/mac/dev_ws/roomie_arm/arm_gazebo/worlds/roomie_aruco_world.world
```

### ArUco 마커 배치 변수들

#### 1. ArUco 마커 101 (첫 번째 마커)
```xml
<include>
  <uri>/home/mac/dev_ws/roomie_arm/install/arm_gazebo/share/arm_gazebo/models/aruco_marker_101</uri>
  <pose>0.3 0.2 0.02 0 1.5708 0</pose>  <!-- 👈 여기서 위치/회전 조절 -->
</include>
```
**조절 변수:**
- `0.3` : X축 위치 (전후 이동)
- `0.2` : Y축 위치 (좌우 이동)  
- `0.02` : Z축 위치 (높이)
- `0 1.5708 0` : 회전각도 (roll pitch yaw)
  - `1.5708` = 90도 (수직 배치, 카메라 보임)
  - `0` = 0도 (수평 배치)

#### 2. ArUco 마커 102 (두 번째 마커)
```xml
<include>
  <uri>/home/mac/dev_ws/roomie_arm/install/arm_gazebo/share/arm_gazebo/models/aruco_marker_102</uri>
  <pose>0.4 -0.15 0.02 0 1.5708 0</pose>  <!-- 👈 여기서 위치/회전 조절 -->
</include>
```

#### 3. ArUco 마커 103 (세 번째 마커)
```xml
<include>
  <uri>/home/mac/dev_ws/roomie_arm/install/arm_gazebo/share/arm_gazebo/models/aruco_marker_103</uri>
  <pose>0.25 -0.3 0.02 0 1.5708 0</pose>  <!-- 👈 여기서 위치/회전 조절 -->
</include>
```

### ArUco 마커 크기 조절

각 마커의 크기는 개별 모델 파일에서 조절:

#### 마커 101 크기 조절
**파일:** `/home/mac/dev_ws/roomie_arm/arm_gazebo/models/aruco_marker_101/model.sdf`
```xml
<collision name="marker_collision">
  <geometry>
    <box>
      <size>0.035 0.035 0.002</size>  <!-- 👈 3.5cm x 3.5cm x 2mm -->
    </box>
  </geometry>
</collision>

<visual name="base">
  <geometry>
    <box>
      <size>0.035 0.035 0.002</size>  <!-- 👈 베이스 크기 -->
    </box>
  </geometry>
</visual>
```

## 🪑 테이블 위치 및 크기 조절

### 파일 위치
```
/home/mac/dev_ws/roomie_arm/arm_gazebo/worlds/roomie_aruco_world.world
```

### 테이블 배치 변수
```xml
<!-- Table -->
<include>
  <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/table</uri>
  <pose>0.5 0 0.4 0 0 0</pose>  <!-- 👈 여기서 테이블 위치 조절 -->
</include>
```

**조절 변수:**
- `0.5` : X축 위치 (로봇 앞뒤 거리)
- `0` : Y축 위치 (로봇 좌우 거리)
- `0.4` : Z축 위치 (테이블 높이)
- `0 0 0` : 회전각도

### 테이블 크기 조절 (커스텀 테이블 필요시)

현재는 Gazebo Fuel의 기본 테이블을 사용중입니다. 크기를 조절하려면 커스텀 테이블 모델을 생성해야 합니다:

```xml
<!-- 커스텀 테이블 예시 -->
<model name="custom_table">
  <static>true</static>
  <link name="table_link">
    <visual name="table_visual">
      <geometry>
        <box>
          <size>1.2 0.8 0.05</size>  <!-- 👈 길이 x 폭 x 두께 -->
        </box>
      </geometry>
    </visual>
  </link>
</model>
```

## 🎮 컨트롤 패널 위치 및 크기 조절

### 파일 위치
```
/home/mac/dev_ws/roomie_arm/arm_gazebo/worlds/roomie_aruco_world.world
```

### 컨트롤 패널 설정 변수
```xml
<!-- Control Panel Base -->
<model name="control_panel">
  <static>true</static>
  <pose>0.35 0 0.08 0 0 0</pose>  <!-- 👈 위치 조절 -->
  <link name="panel_base">
    <collision name="collision">
      <geometry>
        <box>
          <size>0.15 0.25 0.16</size>  <!-- 👈 크기 조절 -->
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>0.15 0.25 0.16</size>  <!-- 👈 크기 조절 -->
        </box>
      </geometry>
      <material>
        <ambient>0.2 0.2 0.2 1</ambient>  <!-- 👈 색상 조절 -->
        <diffuse>0.3 0.3 0.3 1</diffuse>  <!-- 👈 색상 조절 -->
      </material>
    </visual>
  </link>
</model>
```

**설정 변수 설명:**
- **위치 (pose)**: `0.35 0 0.08 0 0 0`
  - `0.35` : X축 (ArUco 마커 뒤쪽 거리)
  - `0` : Y축 (좌우 위치)
  - `0.08` : Z축 (바닥에서 높이)
  - `0 0 0` : 회전각도

- **크기 (size)**: `0.15 0.25 0.16`
  - `0.15` : 폭 (X방향, 15cm)
  - `0.25` : 깊이 (Y방향, 25cm)  
  - `0.16` : 높이 (Z방향, 16cm)

- **색상**: 회색 계열 (ambient/diffuse 0.2~0.3)

**💡 팁:**
- ArUco 마커가 패널 앞면에 부착되므로, 패널을 뒤로 이동시키려면 X값을 증가
- 패널 높이를 조절하면 ArUco 마커들도 함께 높이 조정 필요
- 색상을 밝게 하려면 ambient/diffuse 값을 0.6~0.8로 증가

## 🤖 로봇 스폰 위치 조절

### 파일 위치
```
/home/mac/dev_ws/roomie_arm/arm_gazebo/launch/gazebo.launch.py
```

### 로봇 위치 변수
```python
# 로봇 스폰 위치 (line ~45)
'x': '0.0',      # 👈 X축 위치
'y': '0.0',      # 👈 Y축 위치  
'z': '0.02',     # 👈 Z축 위치 (높이)
'R': '0.0',      # 👈 Roll 회전
'P': '0.0',      # 👈 Pitch 회전
'Y': '0.0',      # 👈 Yaw 회전
```

## 🔄 변경사항 적용 방법

1. **월드 파일 수정 후:**
```bash
cd /home/mac/dev_ws/roomie_arm
colcon build --packages-select arm_gazebo
source install/setup.bash
```

2. **마커 모델 수정 후:**
```bash
colcon build --packages-select arm_gazebo
source install/setup.bash  
```

3. **실행:**
```bash
export GZ_SIM_RESOURCE_PATH=$PWD/install/arm_gazebo/share/arm_gazebo/models:$GZ_SIM_RESOURCE_PATH
ros2 launch arm_gazebo gazebo.launch.py
```

## 📏 좌표계 참고

- **X축**: 앞(+) / 뒤(-)
- **Y축**: 왼쪽(+) / 오른쪽(-)  
- **Z축**: 위(+) / 아래(-)
- **로봇 기준점**: 베이스 중심
- **단위**: 미터(m)

## 💡 팁

1. **ArUco 마커가 카메라에 보이려면:**
   - pitch = 1.5708 (90도 수직)
   - 로봇 전방(+X)에 배치
   
2. **마커 간 거리:**
   - 최소 10cm 이상 떨어뜨려 놓기
   - 카메라 FOV 내에 모두 들어오도록 배치

3. **빠른 테스트:**
   - 월드 파일만 수정하면 바로 적용됨
   - 빌드 후 새로운 터미널에서 실행
