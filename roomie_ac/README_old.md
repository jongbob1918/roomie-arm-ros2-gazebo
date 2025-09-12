
# 1. 동작 원리

## 1️⃣ 단계: 버튼 인식 (Perception)
Vision Service에서 이미지 상의 버튼 bbox의 중심 좌표, 모서리 점들의 좌표, 픽셀 면적을 정규화하여 Arm Controller에 전달

</br>
</br>

## 2️⃣ 단계: 3차원 위치 추정 (Pose Estimation)
2D 좌표를 3D 공간 좌표로 변환하는 과정

### 1) Pose 계산 (from 2D to 3D)
- 실제 버튼 크기(길이 3.5cm), 누르는 자세 거리에서의 버튼 면적 비율을 먼저 측정하여 저장
- cv2.solvePnPRansac 알고리즘을 사용하여 카메라좌표 기준 6D Pose(3차원 위치 tvec과 3차원 방향 rvec)를 계산

### 2) 좌표계 변환 (Coordinate Transformation)
- '카메라 기준' 좌표는 로봇팔이 직접 사용할 수 없기에 사전에 계산된 Hand-Eye Calibration 행렬(T tool→cam)과 로봇팔의 순기구학(Forward Kinematics) 정보(T base→tool)를 이용해, 카메라 기준의 버튼 좌표를 **'로봇 베이스 기준'**의 최종 목표 좌표(T base→btn)로 변환

- 계산식

        (T base→btn)  =  (T base→tool) × (T tool→cam) × (T cam→btn)
​
</br>
</br>

## 3️⃣ 단계: 동작 계획 (Motion Planning)
 최종 목표 좌표로 로봇팔의 엔드 이펙터(끝부분)를 해당 위치와 방향으로 보내기 위해 각 관절의 각도를 계산

### 1) 역기구학 (IK) 해석
- ikpy 라이브러리를 사용하여 목표 위치와 방향을 만족시키는 4개 관절(J1, J2, J3, J4)의 목표 각도(Radian)를 계산

### 2) 각도 변환
- 계산된 라디안(Radian) 단위의 관절각을 ESP32가 이해할 수 있는 서보모터의 각도(Degree, 0°~180°)로 변환 

</br>
</br>

## 4️⃣ 단계: 동작 수행 및 제어 (Execution & Control)
- 최종 서보모터 각도 명령은 시리얼 통신을 통해 ESP32 펌웨어로 전송하여 모터 동작

### 1) 명령 전송 
        <M:90,120,150,30>과 같은 형식의 문자열 명령으로 ESP32에 전송

### 2) 부드러운 움직임 
        ESP32에서  가우시안(Gaussian) 가감속 프로파일을 적용하여 부드러운 움직임 구현. 


</br>
</br>


# 2. 폴더 구조


    roomie_ac/
    ├── package.xml
    ├── CMakeLists.txt
    ├── resource/
    │   └── roomie_ac
    ├── roomie_ac/
    │   ├── __init__.py
    │   ├── ac_node.py                  # 메인 ROS 2 액션 서버 노드
    │   ├── config.py                   # 설정관리파일 
    │   ├── motion_controller.py        # 로봇팔 움직임을 직접 제어
    │   ├── image_servoing.py           # 목표물을 추적/정렬하는 시각 서보잉 로직
    │   ├── kinematics_solver.py        # ikpy를 사용 역기구학(IK)을 계산
    │   ├── coordinate_transformer.py # 2D 카메라 좌표-> 3D 로봇베이스 좌표변환
    │   ├── serial_manager.py           # ESP32와 시리얼 통신
    │   ├── vision_client.py            # Vision Service에 데이터를 요청
    │   └── ros_joint_publisher.py      # 로봇 관절 상태를 RViz2에 발행
    │
    ├── urdf/
    │   └── roomie2.urdf                # 로봇 모델 URDF 파일
    │
    └── launch/
        └── arm_control_launch.py       # 노드를 실행하기 위한 ROS 2 런치 파일




