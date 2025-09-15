#!/usr/bin/env python3
"""
RoomieArm Core 간단한 테스트 클라이언트 (Tkinter 버전)
- ArUco 마커 버튼 클릭 테스트 (101, 102, 103)
- 초기 자세, 관측 자세 이동
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext

from roomiearm_msgs.action import ClickButton
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration


class SimpleTestClient:
    def __init__(self):
        # ROS2 노드 초기화
        rclpy.init()
        self.node = Node('simple_test_client')

        # 액션 클라이언트
        self.button_action_client = ActionClient(self.node, ClickButton, 'click_button')
        self.joint_action_client = ActionClient(self.node, FollowJointTrajectory,
                                               '/joint_trajectory_controller/follow_joint_trajectory')

        # GUI 초기화
        self.setup_gui()

        # ROS2 스피너 스레드
        self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_thread.start()

        self.log_message("🚀 간단한 RoomieArm 테스트 클라이언트 시작")
        self.check_action_servers()

    def setup_gui(self):
        """GUI 설정"""
        self.root = tk.Tk()
        self.root.title("RoomieArm 간단 테스트 클라이언트")
        self.root.geometry("500x600")

        # 제목
        title_label = tk.Label(self.root, text="🤖 RoomieArm Core 테스트",
                              font=("Arial", 16, "bold"), fg="blue")
        title_label.pack(pady=10)

        # 상태 라벨
        self.status_label = tk.Label(self.root, text="상태: 대기 중",
                                    bg="lightgray", relief="sunken")
        self.status_label.pack(fill="x", padx=10, pady=5)

        # ArUco 마커 버튼 프레임
        marker_frame = ttk.LabelFrame(self.root, text="ArUco 마커 버튼 클릭 테스트")
        marker_frame.pack(fill="x", padx=10, pady=10)

        marker_btn_frame = tk.Frame(marker_frame)
        marker_btn_frame.pack(pady=10)

        # 마커 버튼들
        self.marker_buttons = {}
        for i, marker_id in enumerate([101, 102, 103]):
            btn = tk.Button(marker_btn_frame, text=f"마커 {marker_id}",
                           width=12, height=2, bg="lightgreen",
                           command=lambda mid=marker_id: self.click_marker_button(mid))
            btn.grid(row=0, column=i, padx=5)
            self.marker_buttons[marker_id] = btn

        # 제어 버튼 프레임
        control_frame = ttk.LabelFrame(self.root, text="로봇 제어")
        control_frame.pack(fill="x", padx=10, pady=10)

        control_btn_frame = tk.Frame(control_frame)
        control_btn_frame.pack(pady=10)

        # 초기 자세 버튼
        self.home_button = tk.Button(control_btn_frame, text="🏠 초기 자세",
                                    width=15, height=2, bg="lightblue",
                                    command=self.go_to_home)
        self.home_button.grid(row=0, column=0, padx=5)

        # 관측 자세 버튼
        self.observe_button = tk.Button(control_btn_frame, text="👁️ 관측 자세",
                                       width=15, height=2, bg="orange",
                                       command=self.go_to_observation)
        self.observe_button.grid(row=0, column=1, padx=5)

        # 로그 프레임
        log_frame = ttk.LabelFrame(self.root, text="실행 로그")
        log_frame.pack(fill="both", expand=True, padx=10, pady=10)

        # 로그 텍스트
        self.log_text = scrolledtext.ScrolledText(log_frame, height=10,
                                                 bg="black", fg="green",
                                                 font=("Courier", 9))
        self.log_text.pack(fill="both", expand=True, padx=5, pady=5)

        # 하단 버튼들
        bottom_frame = tk.Frame(self.root)
        bottom_frame.pack(fill="x", padx=10, pady=5)

        # 로그 지우기 버튼
        clear_btn = tk.Button(bottom_frame, text="로그 지우기",
                             command=self.clear_log)
        clear_btn.pack(side="left")

        # 종료 버튼
        quit_btn = tk.Button(bottom_frame, text="❌ 종료", bg="red", fg="white",
                            command=self.close_application)
        quit_btn.pack(side="right")

    def spin_ros(self):
        """ROS2 스피너"""
        while rclpy.ok():
            try:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            except Exception as e:
                self.log_message(f"ROS 스핀 오류: {e}")
                break

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
            self.home_button.config(state="disabled")
            self.observe_button.config(state="disabled")

    def disable_marker_buttons(self):
        """마커 버튼 비활성화"""
        for btn in self.marker_buttons.values():
            btn.config(state="disabled")

    def click_marker_button(self, marker_id):
        """마커 버튼 클릭 핸들러"""
        self.log_message(f"🎯 마커 {marker_id} 버튼 클릭 시작")
        self.set_busy_state(True)

        # 액션 goal 생성
        goal_msg = ClickButton.Goal()
        goal_msg.robot_id = 0
        goal_msg.button_id = marker_id

        # 액션 실행 스레드
        action_thread = threading.Thread(
            target=self.execute_button_action,
            args=(goal_msg,),
            daemon=True
        )
        action_thread.start()

    def go_to_home(self):
        """초기 자세로 이동"""
        self.move_to_joint_angles([0.0, 0.0, 0.0, 0.0], "초기 자세")

    def go_to_observation(self):
        """관측 자세로 이동"""
        self.move_to_joint_angles([0.00, 1.19, 1.29, -1.44], "관측 자세")

    def move_to_joint_angles(self, joint_angles, description):
        """지정된 관절 각도로 이동"""
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

        # 액션 실행 스레드
        action_thread = threading.Thread(
            target=self.execute_joint_action,
            args=(goal_msg, description),
            daemon=True
        )
        action_thread.start()

    def execute_button_action(self, goal_msg):
        """버튼 클릭 액션 실행"""
        try:
            self.log_message("액션 전송 중...")

            send_goal_future = self.button_action_client.send_goal_async(goal_msg)

            # Future 대기
            while not send_goal_future.done():
                time.sleep(0.1)

            goal_handle = send_goal_future.result()

            if not goal_handle.accepted:
                self.log_message("❌ 액션이 거부되었습니다")
                self.set_busy_state(False)
                return

            self.log_message("✅ 액션이 수락됨. 실행 중...")

            # 결과 대기
            get_result_future = goal_handle.get_result_async()
            while not get_result_future.done():
                time.sleep(0.1)

            result = get_result_future.result().result

            if result.success:
                self.log_message(f"✅ 성공: {result.message}")
                self.root.after(0, lambda: messagebox.showinfo("성공", f"작업 완료!\n{result.message}"))
            else:
                self.log_message(f"❌ 실패: {result.message}")
                self.root.after(0, lambda: messagebox.showerror("실패", f"작업 실패\n{result.message}"))

        except Exception as e:
            self.log_message(f"❌ 오류 발생: {str(e)}")
            self.root.after(0, lambda: messagebox.showerror("오류", f"오류 발생: {str(e)}"))

        finally:
            self.set_busy_state(False)

    def execute_joint_action(self, goal_msg, description):
        """관절 이동 액션 실행"""
        try:
            self.log_message("관절 이동 액션 전송 중...")

            send_goal_future = self.joint_action_client.send_goal_async(goal_msg)

            while not send_goal_future.done():
                time.sleep(0.1)

            goal_handle = send_goal_future.result()

            if not goal_handle.accepted:
                self.log_message("❌ 관절 이동 액션이 거부되었습니다")
                self.set_busy_state(False)
                return

            self.log_message(f"✅ {description} 이동 중...")

            get_result_future = goal_handle.get_result_async()
            while not get_result_future.done():
                time.sleep(0.1)

            result = get_result_future.result().result
            success = result.error_code == result.SUCCESSFUL

            if success:
                self.log_message(f"✅ {description} 이동 완료")
                self.root.after(0, lambda: messagebox.showinfo("성공", f"{description} 이동 완료!"))
            else:
                self.log_message(f"❌ {description} 이동 실패 (error_code: {result.error_code})")
                self.root.after(0, lambda: messagebox.showerror("실패", f"{description} 이동 실패"))

        except Exception as e:
            self.log_message(f"❌ 관절 이동 오류: {str(e)}")
            self.root.after(0, lambda: messagebox.showerror("오류", f"관절 이동 오류: {str(e)}"))

        finally:
            self.set_busy_state(False)

    def set_busy_state(self, busy):
        """작업 중 상태 설정"""
        def update_gui():
            state = "disabled" if busy else "normal"

            # 모든 버튼 비활성화/활성화
            for btn in self.marker_buttons.values():
                btn.config(state=state)
            self.home_button.config(state=state)
            self.observe_button.config(state=state)

            # 상태 표시
            if busy:
                self.status_label.config(text="상태: 작업 실행 중...", bg="yellow")
            else:
                self.status_label.config(text="상태: 대기 중", bg="lightgray")

        # GUI 업데이트는 메인 스레드에서
        self.root.after(0, update_gui)

    def log_message(self, message):
        """로그 메시지 추가"""
        def update_log():
            timestamp = time.strftime("%H:%M:%S")
            formatted_message = f"[{timestamp}] {message}\n"
            self.log_text.insert(tk.END, formatted_message)
            self.log_text.see(tk.END)

        # GUI 업데이트는 메인 스레드에서
        self.root.after(0, update_log)

    def clear_log(self):
        """로그 지우기"""
        self.log_text.delete(1.0, tk.END)
        self.log_message("로그가 지워졌습니다")

    def close_application(self):
        """애플리케이션 종료"""
        if messagebox.askquestion("종료 확인", "정말로 종료하시겠습니까?") == "yes":
            self.log_message("👋 애플리케이션 종료")
            self.root.quit()
            self.node.destroy_node()
            rclpy.shutdown()

    def run(self):
        """GUI 실행"""
        try:
            self.root.protocol("WM_DELETE_WINDOW", self.close_application)
            self.root.mainloop()
        except KeyboardInterrupt:
            self.close_application()


def main():
    try:
        client = SimpleTestClient()
        client.run()
    except Exception as e:
        print(f"테스트 클라이언트 시작 실패: {e}")
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()