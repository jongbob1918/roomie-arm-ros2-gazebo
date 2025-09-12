import numpy as np
import os

# 'camera_params.npz' 파일이 같은 폴더에 있다고 가정합니다.
current_dir = os.path.dirname(__file__)
npz_file_path = os.path.join(current_dir, 'camera_params.npz')

try:
    with np.load(npz_file_path) as data:
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