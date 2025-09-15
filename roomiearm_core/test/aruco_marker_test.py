import cv2
import numpy as np
import os

# ArUco 사전 및 감지 파라미터 설정
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
aruco_params = cv2.aruco.DetectorParameters()

# DetectorParameters 조정
aruco_params.adaptiveThreshWinSizeMin = 3
aruco_params.adaptiveThreshWinSizeMax = 23
aruco_params.adaptiveThreshWinSizeStep = 10
aruco_params.minMarkerPerimeterRate = 0.04
aruco_params.maxMarkerPerimeterRate = 4.0
aruco_params.polygonalApproxAccuracyRate = 0.03
aruco_params.minCornerDistanceRate = 0.05
aruco_params.minDistanceToBorder = 3
aruco_params.markerBorderBits = 1
aruco_params.errorCorrectionRate = 0.6

# ArUco Detector 초기화
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# 테스트 이미지 폴더 경로 설정
image_folder = "/home/mac/dev_ws/roomiearm-ros2-gazebo/roomiearm_core/test/image"
output_folder = "/home/mac/dev_ws/roomiearm-ros2-gazebo/roomiearm_core/test/output"
os.makedirs(output_folder, exist_ok=True)

# 폴더 내 모든 이미지 파일 검사
for image_name in os.listdir(image_folder):
    image_path = os.path.join(image_folder, image_name)
    cv_image = cv2.imread(image_path)

    if cv_image is None:
        print(f"이미지를 로드할 수 없습니다: {image_path}")
        continue

    # 이미지 전처리
    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    enhanced_image = cv2.equalizeHist(gray_image)

    # ArUco 마커 감지
    corners, ids, _ = aruco_detector.detectMarkers(enhanced_image)

    # 결과 출력
    if ids is not None:
        print(f"이미지: {image_name}, 감지된 마커 개수: {len(ids)}")
        print(f"마커 ID: {ids.flatten().tolist()}")
        for i, marker_id in enumerate(ids):
            print(f"Marker ID: {marker_id[0]}, Corners: {corners[i][0]}")
        # 감지된 마커 시각화
        cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
    else:
        print(f"이미지: {image_name}, 마커를 감지하지 못했습니다.")

    # 결과 이미지 저장
    output_path = os.path.join(output_folder, f"output_{image_name}")
    cv2.imwrite(output_path, cv_image)
    print(f"결과 이미지를 저장했습니다: {output_path}")