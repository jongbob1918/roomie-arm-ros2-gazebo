import cv2
import numpy as np
from cv2 import aruco

# ArUco Dictionary 선택 (예: DICT_4X4_250)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

# 생성할 마커 ID (101, 102, 103)
marker_ids = [101, 102, 103]

# 마커 크기 (픽셀 단위)
marker_size = 200  # 200x200 픽셀

# 테두리 크기 설정
border_color = 255  # 하얀색 테두리

# 마커 생성 및 저장
for marker_id in marker_ids:
    marker_image = aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
    


    file_name = f"marker_{marker_id}_white_border.png"
    cv2.imwrite(file_name, marker_image)
    print(f"Marker {marker_id} saved as {file_name}")