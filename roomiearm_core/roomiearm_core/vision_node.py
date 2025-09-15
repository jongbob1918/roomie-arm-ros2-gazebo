# ~/dev_ws/roomiearm-ros2-gazebo/roomiearm_core/roomiearm_core/vision_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

from roomiearm_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # 이미지 토픽을 구독(subscribe)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # 표준 카메라 토픽 이름
            self.image_callback,
            100)
        
        # 마커 배열 토픽을 발행(publish)
        self.publisher_ = self.create_publisher(MarkerArray, '/marker_array', 10)
        
        # ROS 이미지 메시지를 OpenCV 이미지로 변환하기 위한 CvBridge
        self.bridge = CvBridge()

        # ArUco 사전(dictionary) 및 파라미터 설정
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.aruco_params = cv2.aruco.DetectorParameters()

        self.get_logger().info('Vision node has been started.')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w, _ = cv_image.shape
            self.get_logger().info(f"Image received: {h}x{w}")
            self.get_logger().info(f"Aruco Dict: {self.aruco_dict}, Aruco Params: {self.aruco_params}")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # ArUco Detector 초기화
        aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # ArUco 마커 감지
        corners, ids, _ = aruco_detector.detectMarkers(cv_image)

        if ids is not None:
            self.get_logger().info(f"Detected {len(ids)} markers: {ids.flatten().tolist()}")
            for i, marker_id in enumerate(ids):
                self.get_logger().info(f"Marker ID: {marker_id[0]}, Corners: {corners[i][0]}")
        else:
            self.get_logger().info("No markers detected.")

        # 발행할 MarkerArray 메시지 생성
        marker_array_msg = MarkerArray()
        marker_array_msg.header = msg.header  # 수신한 이미지의 헤더를 그대로 사용

        if ids is not None:
            # 감지된 모든 마커에 대해 반복
            for i, marker_id in enumerate(ids):
                marker_corners = corners[i][0]

                # Marker 메시지 생성
                marker_msg = Marker()
                marker_msg.marker_id = int(marker_id[0])

                # 중심점 계산 및 정규화
                center_x = np.mean([c[0] for c in marker_corners])
                center_y = np.mean([c[1] for c in marker_corners])

                marker_msg.center.x = center_x / w
                marker_msg.center.y = center_y / h
                marker_msg.center.z = 0.0  # 사용 안 함

                # 네 꼭짓점 좌표 정규화
                corner_points = []
                for corner in marker_corners:
                    point = Point()
                    point.x = float(corner[0]) / w
                    point.y = float(corner[1]) / h
                    point.z = 0.0  # 사용 안 함
                    corner_points.append(point)

                # Point[4] 타입에 맞게 변환
                marker_msg.corners = corner_points

                # 최종 마커 배열에 추가
                marker_array_msg.markers.append(marker_msg)

        # 마커가 하나라도 감지되었으면 메시지 발행
        if len(marker_array_msg.markers) > 0:
            self.publisher_.publish(marker_array_msg)


def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    rclpy.spin(vision_node)
    vision_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()