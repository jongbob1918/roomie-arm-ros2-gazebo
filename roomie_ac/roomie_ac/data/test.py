import numpy as np

# 'camera_params.npz' 파일이 같은 폴더에 있다고 가정합니다.
try:
    with np.load('/home/mac/dev_ws/addinedu/project/ros-repo-2/ros2_ws/src/roomie_ac/roomie_ac/data/camera_params.npz') as data:
        mtx = data['mtx']
        dist = data['dist']
        
        print("--- 카메라 행렬 (mtx) ---")
        print(mtx)
        
        print("\n--- 왜곡 계수 (dist) ---")
        print(dist)
        
        fx = mtx[0, 0]
        print(f"\n추출된 fx 값: {fx}")

except FileNotFoundError:
    print("오류: 'camera_params.npz' 파일을 찾을 수 없습니다.")