#!/usr/bin/env python3
"""
RoomieArm Core GUI 테스트 클라이언트
- ArUco 마커 버튼 클릭 테스트 (101, 102, 103)
- 초기 자세 이동
- 실시간 상태 표시
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import threading
import time

from PyQt6.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout,
                            QWidget, QPushButton, QLabel, QTextEdit, QGroupBox,
                            QProgressBar, QStatusBar, QMessageBox)
from PyQt6.QtCore import QTimer, pyqtSignal, QThread, pyqtSlot
from PyQt6.QtGui import QFont, QIcon

from roomiearm_msgs.action import ClickButton
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration


class ActionThread(QThread):
    """액션 실행을 위한 별도 스레드"""
    finished = pyqtSignal(bool, str)  # success, message
    feedback_received = pyqtSignal(str)  # feedback message

    def __init__(self, action_client, goal_msg):
        super().__init__()
        self.action_client = action_client
        self.goal_msg = goal_msg

    def run(self):
        """액션 실행"""
        try:
            # 액션 전송
            send_goal_future = self.action_client.send_goal_async(
                self.goal_msg,
                feedback_callback=self.feedback_callback
            )

            # rclpy.spin을 별도로 처리
            while not send_goal_future.done():
                rclpy.spin_once(self.action_client._node, timeout_sec=0.1)

            goal_handle = send_goal_future.result()

            if not goal_handle.accepted:
                self.finished.emit(False, "액션이 거부되었습니다")
                return

            self.feedback_received.emit("액션이 수락됨. 실행 중...")

            # 결과 대기
            get_result_future = goal_handle.get_result_async()
            while not get_result_future.done():
                rclpy.spin_once(self.action_client._node, timeout_sec=0.1)

            result = get_result_future.result().result
            self.finished.emit(result.success, result.message)

        except Exception as e:
            self.finished.emit(False, f"오류 발생: {str(e)}")

    def feedback_callback(self, feedback_msg):
        """피드백 콜백"""
        feedback = feedback_msg.feedback
        self.feedback_received.emit(f"Robot {feedback.robot_id}: {feedback.status}")


class RoomieArmTestGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        # ROS2 노드 초기화
        rclpy.init()
        self.node = Node('roomiearm_test_gui')

        # 액션 클라이언트
        self.button_action_client = ActionClient(self.node, ClickButton, 'click_button')
        self.joint_action_client = ActionClient(self.node, FollowJointTrajectory,
                                               '/joint_trajectory_controller/follow_joint_trajectory')

        # GUI 초기화
        self.init_ui()

        # ROS2 스피너 타이머
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(50)  # 20Hz

        # 액션 스레드
        self.action_thread = None

        self.log_message("🚀 RoomieArm 테스트 GUI 시작")
        self.check_action_servers()

    def init_ui(self):
        """UI 초기화"""
        self.setWindowTitle("RoomieArm Core 테스트 클라이언트")
        self.setGeometry(100, 100, 600, 700)

        # 중앙 위젯
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # 제목
        title_label = QLabel("🤖 RoomieArm Core 테스트")
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setStyleSheet("color: #2E86AB; margin: 10px;")
        layout.addWidget(title_label)

        # 상태 표시
        self.status_label = QLabel("상태: 대기 중")
        self.status_label.setStyleSheet("background-color: #F0F0F0; padding: 5px; border-radius: 3px;")
        layout.addWidget(self.status_label)

        # 진행 상황 바
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        layout.addWidget(self.progress_bar)

        # ArUco 마커 버튼 그룹
        marker_group = QGroupBox("ArUco 마커 버튼 클릭 테스트")
        marker_layout = QHBoxLayout(marker_group)

        self.marker_buttons = {}
        for marker_id in [101, 102, 103]:
            btn = QPushButton(f"마커 {marker_id}")
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #4CAF50;
                    color: white;
                    border: none;
                    padding: 10px;
                    border-radius: 5px;
                    font-size: 12px;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background-color: #45a049;
                }
                QPushButton:pressed {
                    background-color: #3d8b40;
                }
                QPushButton:disabled {
                    background-color: #cccccc;
                }
            """)
            btn.clicked.connect(lambda checked, mid=marker_id: self.click_marker_button(mid))
            marker_layout.addWidget(btn)
            self.marker_buttons[marker_id] = btn

        layout.addWidget(marker_group)

        # 제어 버튼 그룹
        control_group = QGroupBox("로봇 제어")
        control_layout = QHBoxLayout(control_group)

        # 초기 자세 버튼
        self.home_button = QPushButton("🏠 초기 자세")
        self.home_button.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                border: none;
                padding: 10px;
                border-radius: 5px;
                font-size: 12px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
            QPushButton:disabled {
                background-color: #cccccc;
            }
        """)
        self.home_button.clicked.connect(self.go_to_home)
        control_layout.addWidget(self.home_button)

        # 관측 자세 버튼
        self.observe_button = QPushButton("👁️ 관측 자세")
        self.observe_button.setStyleSheet("""
            QPushButton {
                background-color: #FF9800;
                color: white;
                border: none;
                padding: 10px;
                border-radius: 5px;
                font-size: 12px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #F57C00;
            }
            QPushButton:disabled {
                background-color: #cccccc;
            }
        """)
        self.observe_button.clicked.connect(self.go_to_observation)
        control_layout.addWidget(self.observe_button)

        layout.addWidget(control_group)

        # 로그 창
        log_group = QGroupBox("실행 로그")
        log_layout = QVBoxLayout(log_group)

        self.log_text = QTextEdit()
        self.log_text.setMaximumHeight(200)
        self.log_text.setStyleSheet("background-color: #1E1E1E; color: #00FF00; font-family: monospace;")
        log_layout.addWidget(self.log_text)

        # 로그 지우기 버튼
        clear_log_btn = QPushButton("로그 지우기")
        clear_log_btn.clicked.connect(self.clear_log)
        log_layout.addWidget(clear_log_btn)

        layout.addWidget(log_group)

        # 닫기 버튼
        self.close_button = QPushButton("❌ 종료")
        self.close_button.setStyleSheet("""
            QPushButton {
                background-color: #F44336;
                color: white;
                border: none;
                padding: 10px;
                border-radius: 5px;
                font-size: 12px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #D32F2F;
            }
        """)
        self.close_button.clicked.connect(self.close_application)
        layout.addWidget(self.close_button)

        # 상태바
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("준비됨")

    def spin_ros(self):
        """ROS2 스피너"""
        try:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        except Exception as e:
            self.log_message(f"ROS 스핀 오류: {e}")

    def check_action_servers(self):
        """액션 서버 확인"""
        self.log_message("액션 서버 확인 중...")

        # Button action server 확인
        if self.button_action_client.wait_for_server(timeout_sec=2.0):
            self.log_message("✅ Button Click 액션 서버 연결됨")
        else:
            self.log_message("❌ Button Click 액션 서버를 찾을 수 없음")
            self.disable_marker_buttons()

        # Joint trajectory server 확인
        if self.joint_action_client.wait_for_server(timeout_sec=2.0):
            self.log_message("✅ Joint Trajectory 액션 서버 연결됨")
        else:
            self.log_message("❌ Joint Trajectory 액션 서버를 찾을 수 없음")
            self.home_button.setEnabled(False)
            self.observe_button.setEnabled(False)

    def disable_marker_buttons(self):
        """마커 버튼 비활성화"""
        for btn in self.marker_buttons.values():
            btn.setEnabled(False)

    def click_marker_button(self, marker_id):
        """마커 버튼 클릭 핸들러"""
        if self.action_thread and self.action_thread.isRunning():
            self.log_message("⚠️ 이미 다른 작업이 실행 중입니다")
            return

        self.log_message(f"🎯 마커 {marker_id} 버튼 클릭 시작")
        self.set_busy_state(True)

        # 액션 goal 생성
        goal_msg = ClickButton.Goal()
        goal_msg.robot_id = 0
        goal_msg.button_id = marker_id

        # 액션 스레드 시작
        self.action_thread = ActionThread(self.button_action_client, goal_msg)
        self.action_thread.finished.connect(self.on_action_finished)
        self.action_thread.feedback_received.connect(self.on_feedback_received)
        self.action_thread.start()

    def go_to_home(self):
        """초기 자세로 이동"""
        self.move_to_joint_angles([0.0, 0.0, 0.0, 0.0], "초기 자세")

    def go_to_observation(self):
        """관측 자세로 이동"""
        self.move_to_joint_angles([0.00, 1.19, 1.29, -1.44], "관측 자세")

    def move_to_joint_angles(self, joint_angles, description):
        """지정된 관절 각도로 이동"""
        if self.action_thread and self.action_thread.isRunning():
            self.log_message("⚠️ 이미 다른 작업이 실행 중입니다")
            return

        self.log_message(f"🤖 {description}로 이동 시작")
        self.set_busy_state(True)

        # Joint trajectory goal 생성
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']

        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start = Duration(sec=3, nanosec=0)
        goal_msg.trajectory.points.append(point)

        # 액션 스레드 시작 (Joint trajectory용)
        self.action_thread = JointActionThread(self.joint_action_client, goal_msg, description)
        self.action_thread.finished.connect(self.on_action_finished)
        self.action_thread.feedback_received.connect(self.on_feedback_received)
        self.action_thread.start()

    @pyqtSlot(bool, str)
    def on_action_finished(self, success, message):
        """액션 완료 시 호출"""
        self.set_busy_state(False)

        if success:
            self.log_message(f"✅ 성공: {message}")
            self.status_bar.showMessage("작업 완료")
            QMessageBox.information(self, "성공", f"작업이 완료되었습니다!\n{message}")
        else:
            self.log_message(f"❌ 실패: {message}")
            self.status_bar.showMessage("작업 실패")
            QMessageBox.warning(self, "실패", f"작업이 실패했습니다.\n{message}")

    @pyqtSlot(str)
    def on_feedback_received(self, feedback_message):
        """피드백 수신 시 호출"""
        self.log_message(f"📡 {feedback_message}")
        self.status_bar.showMessage(feedback_message)

    def set_busy_state(self, busy):
        """작업 중 상태 설정"""
        # 모든 버튼 비활성화/활성화
        for btn in self.marker_buttons.values():
            btn.setEnabled(not busy)
        self.home_button.setEnabled(not busy)
        self.observe_button.setEnabled(not busy)

        # 진행 상황 바 표시/숨김
        self.progress_bar.setVisible(busy)
        if busy:
            self.progress_bar.setRange(0, 0)  # 무한 진행
            self.status_label.setText("상태: 작업 실행 중...")
            self.status_label.setStyleSheet("background-color: #FFF3CD; padding: 5px; border-radius: 3px;")
        else:
            self.progress_bar.setRange(0, 100)
            self.progress_bar.setValue(100)
            self.status_label.setText("상태: 대기 중")
            self.status_label.setStyleSheet("background-color: #F0F0F0; padding: 5px; border-radius: 3px;")

    def log_message(self, message):
        """로그 메시지 추가"""
        timestamp = time.strftime("%H:%M:%S")
        formatted_message = f"[{timestamp}] {message}"
        self.log_text.append(formatted_message)

        # 자동 스크롤
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def clear_log(self):
        """로그 지우기"""
        self.log_text.clear()
        self.log_message("로그가 지워졌습니다")

    def close_application(self):
        """애플리케이션 종료"""
        reply = QMessageBox.question(self, '종료 확인',
                                   '정말로 애플리케이션을 종료하시겠습니까?',
                                   QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)

        if reply == QMessageBox.StandardButton.Yes:
            self.log_message("👋 애플리케이션 종료")
            self.close()

    def closeEvent(self, event):
        """창 닫기 이벤트"""
        if self.action_thread and self.action_thread.isRunning():
            self.action_thread.wait()

        self.ros_timer.stop()
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()


class JointActionThread(QThread):
    """관절 이동 액션을 위한 별도 스레드"""
    finished = pyqtSignal(bool, str)
    feedback_received = pyqtSignal(str)

    def __init__(self, action_client, goal_msg, description):
        super().__init__()
        self.action_client = action_client
        self.goal_msg = goal_msg
        self.description = description

    def run(self):
        """액션 실행"""
        try:
            send_goal_future = self.action_client.send_goal_async(self.goal_msg)

            while not send_goal_future.done():
                rclpy.spin_once(self.action_client._node, timeout_sec=0.1)

            goal_handle = send_goal_future.result()

            if not goal_handle.accepted:
                self.finished.emit(False, "관절 이동 액션이 거부되었습니다")
                return

            self.feedback_received.emit(f"{self.description} 이동 중...")

            get_result_future = goal_handle.get_result_async()
            while not get_result_future.done():
                rclpy.spin_once(self.action_client._node, timeout_sec=0.1)

            result = get_result_future.result().result
            success = result.error_code == result.SUCCESSFUL

            if success:
                self.finished.emit(True, f"{self.description} 이동 완료")
            else:
                self.finished.emit(False, f"{self.description} 이동 실패 (error_code: {result.error_code})")

        except Exception as e:
            self.finished.emit(False, f"관절 이동 오류: {str(e)}")


def main():
    app = QApplication(sys.argv)

    # 애플리케이션 스타일 설정
    app.setStyle('Fusion')

    try:
        window = RoomieArmTestGUI()
        window.show()
        sys.exit(app.exec_())
    except Exception as e:
        print(f"GUI 시작 실패: {e}")
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()