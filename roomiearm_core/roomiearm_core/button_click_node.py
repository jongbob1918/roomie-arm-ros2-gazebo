import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
import cv2
import numpy as np
import time
import math

# TF2 관련 임포트
import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped
from geometry_msgs.msg import PoseStamped, Quaternion
from scipy.spatial.transform import Rotation as R

# ROS 메시지 타입 임포트
from roomiearm_msgs.action import ClickButton
from roomiearm_msgs.msg import MarkerArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

# 재사용할 로직 임포트
from .kinematics_solver import DirectTransformIK

# 설정 상수 정의
CAMERA_MATRIX = np.array([
    [693.32, 0, 300.69],
    [0, 692.29, 281.98],
    [0, 0, 1]], dtype=np.float32)

DIST_COEFFS = np.array([
    -0.4149, 0.2780, 0.0004, -0.0002, -0.1941
], dtype=np.float32)

# 이미지 해상도
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

# 마커 설정
MARKER_SIZE = 0.035  # 3.5cm

# 접근 거리
APPROACH_DISTANCE = 0.05   # 5cm
PRESS_DISTANCE = 0.005     # 0.5cm

# 원추형 접근 설정
MAX_APPROACH_ANGLE = 30    # degrees
CONE_ANGLE_STEP = 10       # degrees
CONE_PHI_STEP = 30         # degrees

# 조인트 제한
JOINT_LIMITS_MIN = [-math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2]
JOINT_LIMITS_MAX = [math.pi/2, math.pi/2, math.pi/2, math.pi/2]

# 작업공간 제한
WORKSPACE_RADIUS_MIN = 0.05  # 5cm
WORKSPACE_RADIUS_MAX = 0.34  # 34cm
WORKSPACE_HEIGHT_MIN = 0.05  # 5cm
WORKSPACE_HEIGHT_MAX = 0.50  # 50cm

# 자세 정의
INITIAL_POSE = [0.0, 0.0, 0.0, 0.0]
OBSERVATION_POSE = [0.0, -0.7, 1.0, 0.6]
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4']


class ButtonClickNode(Node):
    def __init__(self):
        super().__init__('button_click_node')

        # 실시간 조인트 상태 추적
        self.joint_state_subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.current_joint_states = INITIAL_POSE.copy()

        # 카메라 파라미터 설정
        self.camera_matrix = CAMERA_MATRIX
        self.dist_coeffs = DIST_COEFFS
        self.image_width = IMAGE_WIDTH
        self.image_height = IMAGE_HEIGHT

        # ROS 통신 설정
        self._action_server = ActionServer(
            self, ClickButton, 'click_button', self.execute_callback)
        self._jnt_traj_cli = ActionClient(
            self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self.marker_subscription = self.create_subscription(
            MarkerArray, 'marker_array', self.marker_callback, 10)

        # TF2 리스너 초기화
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 키네마틱스 솔버
        self.ik_solver = DirectTransformIK()

        # 마커 데이터 저장
        self.latest_marker_array = None

        self.get_logger().info('Button Click Node initialized')


    def joint_state_callback(self, msg):
        """ros2_control에서 현재 조인트 상태 수신"""
        try:
            joint_positions = []
            for joint_name in JOINT_NAMES:
                idx = msg.name.index(joint_name)
                joint_positions.append(msg.position[idx])
            self.current_joint_states = joint_positions
        except ValueError as e:
            self.get_logger().warn(f'Joint state parsing failed: {e}')


    def marker_callback(self, msg):
        """마커 배열 수신"""
        self.latest_marker_array = msg


    def transform_marker_to_base(self, marker_pose_camera):
        """camera_link → ee_link → base_link"""
        try:
            # Step 1: camera_link → ee_link
            transform_cam_to_ee = self.tf_buffer.lookup_transform(
                'ee_link', 'camera_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            marker_pose_ee = do_transform_pose_stamped(marker_pose_camera, transform_cam_to_ee)

            # Step 2: ee_link → base_link
            transform_ee_to_base = self.tf_buffer.lookup_transform(
                'base_link', 'ee_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            marker_pose_base = do_transform_pose_stamped(marker_pose_ee, transform_ee_to_base)

            return marker_pose_base

        except Exception as e:
            self.get_logger().error(f'TF2 transform failed: {e}')
            return None

    def generate_cone_approach_candidates(self, ideal_approach):
        """원추형 영역에서 접근 방향 후보 생성"""
        candidates = [ideal_approach]  # 중심축

        for theta in range(CONE_ANGLE_STEP, MAX_APPROACH_ANGLE + 1, CONE_ANGLE_STEP):
            for phi in range(0, 360, CONE_PHI_STEP):
                # 구면 좌표계 → 직교 좌표계
                theta_rad, phi_rad = np.radians(theta), np.radians(phi)
                local_dir = np.array([
                    np.sin(theta_rad) * np.cos(phi_rad),
                    np.sin(theta_rad) * np.sin(phi_rad),
                    np.cos(theta_rad)
                ])

                # 글로벌 좌표계로 변환
                z_axis = ideal_approach / np.linalg.norm(ideal_approach)
                x_axis = np.cross([0, 0, 1] if abs(z_axis[2]) < 0.9 else [1, 0, 0], z_axis)
                x_axis = x_axis / np.linalg.norm(x_axis)
                y_axis = np.cross(z_axis, x_axis)

                rotation_matrix = np.column_stack([x_axis, y_axis, z_axis])
                global_direction = rotation_matrix @ local_dir
                candidates.append(global_direction)

        return candidates

    def filter_by_workspace(self, approach_candidates, marker_pos):
        """작업공간 내 후보만 선별"""
        valid_approaches = []

        for approach_dir in approach_candidates:
            approach_pos = marker_pos + approach_dir * APPROACH_DISTANCE
            press_pos = marker_pos + approach_dir * PRESS_DISTANCE

            if (self.is_in_workspace(approach_pos) and self.is_in_workspace(press_pos)):
                valid_approaches.append((approach_dir, approach_pos, press_pos))

        return valid_approaches

    def is_in_workspace(self, position):
        """작업공간 검사"""
        x, y, z = position
        radius = math.sqrt(x**2 + y**2)
        return (WORKSPACE_RADIUS_MIN <= radius <= WORKSPACE_RADIUS_MAX and
                WORKSPACE_HEIGHT_MIN <= z <= WORKSPACE_HEIGHT_MAX)

    def generate_and_filter_approaches(self, marker_pose_base):
        """접근 후보 생성 및 필터링"""
        marker_pos = np.array([
            marker_pose_base.pose.position.x,
            marker_pose_base.pose.position.y,
            marker_pose_base.pose.position.z
        ])

        # 마커 법선 벡터 (이상적 접근 방향)
        marker_quat = [
            marker_pose_base.pose.orientation.x,
            marker_pose_base.pose.orientation.y,
            marker_pose_base.pose.orientation.z,
            marker_pose_base.pose.orientation.w
        ]
        marker_rotation = R.from_quat(marker_quat)
        ideal_approach = marker_rotation.as_matrix()[:, 2]  # Z축 (법선)

        # 원추형 접근 후보 생성
        approach_candidates = self.generate_cone_approach_candidates(ideal_approach)

        # 작업공간 필터링
        valid_approaches = self.filter_by_workspace(approach_candidates, marker_pos)

        return valid_approaches

    def detect_and_transform_marker(self, target_id):
        """마커 감지 및 좌표 변환"""
        # 마커 데이터 대기
        wait_start_time = self.get_clock().now()
        while self.latest_marker_array is None:
            if (self.get_clock().now() - wait_start_time).nanoseconds > 3 * 1e9:
                raise Exception("Marker detection timeout")
            time.sleep(0.1)

        # 목표 마커 찾기
        found_marker = None
        for marker in self.latest_marker_array.markers:
            if marker.marker_id == target_id:
                found_marker = marker
                break

        if not found_marker:
            raise Exception(f"Target marker ID {target_id} not found")

        # solvePnP로 3D 자세 추정
        object_points = np.array([
            [-MARKER_SIZE / 2, MARKER_SIZE / 2, 0],
            [MARKER_SIZE / 2, MARKER_SIZE / 2, 0],
            [MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
            [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0]
        ], dtype=np.float32)

        image_points = np.array([
            (c.x * self.image_width, c.y * self.image_height) for c in found_marker.corners
        ], dtype=np.float32)

        success, rvec, tvec = cv2.solvePnP(
            object_points, image_points, self.camera_matrix, self.dist_coeffs)

        if not success:
            raise Exception("solvePnP failed to estimate pose")

        # 카메라 프레임에서 PoseStamped 생성
        camera_frame_id = self.latest_marker_array.header.frame_id
        marker_pose_camera = PoseStamped()
        marker_pose_camera.header.frame_id = camera_frame_id
        marker_pose_camera.header.stamp = self.get_clock().now().to_msg()
        marker_pose_camera.pose.position.x = float(tvec[0][0])
        marker_pose_camera.pose.position.y = float(tvec[1][0])
        marker_pose_camera.pose.position.z = float(tvec[2][0])

        # Rodrigues → Quaternion
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        r = R.from_matrix(rotation_matrix)
        q = r.as_quat()  # [x, y, z, w]
        marker_pose_camera.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # TF2 변환
        marker_pose_base = self.transform_marker_to_base(marker_pose_camera)
        if marker_pose_base is None:
            raise Exception("TF2 transformation failed")

        return marker_pose_base

    def move_to_joint_angles(self, joint_angles, duration_sec=2.0):
        """조인트 각도로 이동"""
        if not self._jnt_traj_cli.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Joint trajectory controller not available')
            return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9)
        )
        goal_msg.trajectory.points.append(point)

        self.get_logger().info(f"Moving to joints: {[f'{a:.3f}' for a in joint_angles]}")
        send_goal_future = self._jnt_traj_cli.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        success = result.error_code == result.SUCCESSFUL
        if not success:
            self.get_logger().error(f'Trajectory failed: error_code={result.error_code}')

        return success

    def execute_sequence(self, joints_approach, joints_press, joints_retract):
        """3자세 순차 실행"""
        return (self.move_to_joint_angles(joints_approach, 2.0) and
                self.move_to_joint_angles(joints_press, 1.5) and
                self.move_to_joint_angles(joints_retract, 1.5))

    def execute_callback(self, goal_handle):
        """배치 IK 계산 방식"""
        target_id = goal_handle.request.button_id
        self.get_logger().info(f"===== Button Click Sequence for ID: {target_id} =====")

        try:
            # 1. 관측 자세로 이동
            self.get_logger().info("Step 1: Moving to observation pose...")
            self.latest_marker_array = None
            if not self.move_to_joint_angles(OBSERVATION_POSE, 3.0):
                raise Exception("Failed to move to observation pose")

            time.sleep(0.5)  # 안정화

            # 2. 마커 감지 및 변환
            self.get_logger().info("Step 2: Detecting and transforming marker...")
            marker_pose_base = self.detect_and_transform_marker(target_id)

            self.get_logger().info(
                f"Marker pose in base_link: "
                f"x={marker_pose_base.pose.position.x:.3f}, "
                f"y={marker_pose_base.pose.position.y:.3f}, "
                f"z={marker_pose_base.pose.position.z:.3f}"
            )

            # 3. 원추형 접근 후보 생성 및 필터링
            self.get_logger().info("Step 3: Generating approach candidates...")
            approach_candidates = self.generate_and_filter_approaches(marker_pose_base)

            if not approach_candidates:
                raise Exception("No valid approach candidates found")

            self.get_logger().info(f"Found {len(approach_candidates)} valid approaches")

            # 4. 배치 IK 계산 (current_joint_states 기준)
            self.get_logger().info("Step 4: Solving IK for approach candidates...")
            for i, (approach_dir, approach_pos, press_pos) in enumerate(approach_candidates):
                try:
                    self.get_logger().info(f"Trying approach {i+1}/{len(approach_candidates)}")

                    joints_approach = self.ik_solver.inverse_kinematics(
                        approach_pos.tolist(), self.current_joint_states
                    )
                    joints_press = self.ik_solver.inverse_kinematics(
                        press_pos.tolist(), self.current_joint_states
                    )
                    joints_retract = joints_approach  # 후퇴 = 접근

                    # 5. 순차 실행
                    self.get_logger().info("Step 5: Executing button press sequence...")
                    success = self.execute_sequence(joints_approach, joints_press, joints_retract)

                    if success:
                        # 6. 초기 자세로 복귀
                        self.get_logger().info("Step 6: Returning to initial pose...")
                        if self.move_to_joint_angles(INITIAL_POSE, 3.0):
                            self.get_logger().info(f"===== Button Click for ID {target_id} SUCCEEDED =====")
                            goal_handle.succeed()
                            return ClickButton.Result(success=True, message="Button click completed successfully")

                except Exception as e:
                    self.get_logger().warn(f'Approach {i+1} failed: {e}')
                    continue

            # 모든 접근 실패
            raise Exception("All approach attempts failed")

        except Exception as e:
            error_msg = f"Button click failed: {e}"
            self.get_logger().error(error_msg)
            goal_handle.abort()
            return ClickButton.Result(success=False, message=error_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ButtonClickNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()