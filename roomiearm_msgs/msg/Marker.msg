# Marker.msg
# 단일 ArUco 마커의 감지 정보

int32 marker_id          # 감지된 마커의 ID

# 0~1 사이로 정규화된 좌표
geometry_msgs/Point center      # 마커의 중심점 (x, y 값 사용)
geometry_msgs/Point[4] corners   # 4개의 꼭짓점 (x, y 값 사용)