#!/usr/bin/env python3
"""
Roomie 4DOF Robot GUI Controller
슬라이더를 통한 실시간 조인트 제어
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import tkinter as tk
from tkinter import ttk
import threading
import time


class RobotGUIController(Node):
    def __init__(self):
        super().__init__('robot_gui_controller')
        
        # Publisher 생성
        self.command_pub = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            10
        )
        
        # 현재 조인트 위치
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]
        self.joint_names = ['Joint 1 (Base)', 'Joint 2 (Shoulder)', 'Joint 3 (Elbow)', 'Joint 4 (Wrist)']
        
        # GUI 초기화
        self.init_gui()
        
        self.get_logger().info('Robot GUI Controller started!')
        
    def init_gui(self):
        """GUI 인터페이스 생성"""
        self.root = tk.Tk()
        self.root.title("🤖 Roomie 4DOF Robot Controller")
        self.root.geometry("500x400")
        self.root.resizable(True, True)
        
        # 메인 프레임
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 제목
        title_label = ttk.Label(main_frame, text="🤖 Roomie 4DOF Robot Controller", 
                               font=('Arial', 14, 'bold'))
        title_label.grid(row=0, column=0, columnspan=3, pady=(0, 20))
        
        # 슬라이더들
        self.sliders = []
        self.value_labels = []
        
        for i, joint_name in enumerate(self.joint_names):
            # 조인트 이름 레이블
            name_label = ttk.Label(main_frame, text=joint_name, font=('Arial', 10, 'bold'))
            name_label.grid(row=i+1, column=0, sticky=tk.W, pady=5)
            
            # 슬라이더
            slider = tk.Scale(main_frame, 
                            from_=-1.54, to=1.54, 
                            resolution=0.01,
                            orient=tk.HORIZONTAL,
                            length=250,
                            command=lambda val, idx=i: self.slider_changed(idx, val))
            slider.set(0.0)
            slider.grid(row=i+1, column=1, padx=10, pady=5)
            self.sliders.append(slider)
            
            # 값 표시 레이블
            value_label = ttk.Label(main_frame, text="0.00", font=('Arial', 10))
            value_label.grid(row=i+1, column=2, sticky=tk.W, pady=5)
            self.value_labels.append(value_label)
        
        # 버튼들
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=6, column=0, columnspan=3, pady=20)
        
        # Home 버튼
        home_button = ttk.Button(button_frame, text="🏠 Home Position", 
                               command=self.home_position)
        home_button.grid(row=0, column=0, padx=5)
        
        # Stop 버튼
        stop_button = ttk.Button(button_frame, text="⏹️ Stop", 
                               command=self.stop_robot)
        stop_button.grid(row=0, column=1, padx=5)
        
        # 상태 표시
        self.status_label = ttk.Label(main_frame, text="Ready", 
                                     font=('Arial', 10), foreground='green')
        self.status_label.grid(row=7, column=0, columnspan=3, pady=10)
        
        # 사용법 안내
        help_text = ("사용법: 슬라이더를 움직여서 조인트를 제어하세요\n"
                    "범위: -1.54 ~ 1.54 라디안 (약 -88° ~ 88°)")
        help_label = ttk.Label(main_frame, text=help_text, 
                              font=('Arial', 9), foreground='gray')
        help_label.grid(row=8, column=0, columnspan=3, pady=10)
        
        # 윈도우 닫기 이벤트
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
    def slider_changed(self, joint_idx, value):
        """슬라이더 값 변경 시 호출"""
        try:
            joint_value = float(value)
            self.joint_positions[joint_idx] = joint_value
            
            # 값 레이블 업데이트
            self.value_labels[joint_idx].config(text=f"{joint_value:.2f}")
            
            # 로봇에 명령 전송
            self.send_command()
            
            # 상태 업데이트
            self.status_label.config(text=f"Moving Joint {joint_idx+1}", foreground='blue')
            
        except Exception as e:
            self.get_logger().error(f"Slider error: {e}")
    
    def send_command(self):
        """조인트 명령 전송"""
        try:
            msg = Float64MultiArray()
            msg.data = self.joint_positions[:]
            self.command_pub.publish(msg)
            
            self.get_logger().debug(
                f'Sent: [{self.joint_positions[0]:.2f}, {self.joint_positions[1]:.2f}, '
                f'{self.joint_positions[2]:.2f}, {self.joint_positions[3]:.2f}]'
            )
            
        except Exception as e:
            self.get_logger().error(f"Command send error: {e}")
    
    def home_position(self):
        """홈 포지션으로 이동"""
        for i, slider in enumerate(self.sliders):
            slider.set(0.0)
        
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]
        self.send_command()
        
        # 값 레이블들 업데이트
        for label in self.value_labels:
            label.config(text="0.00")
            
        self.status_label.config(text="Home Position", foreground='green')
    
    def stop_robot(self):
        """현재 위치에서 정지"""
        self.send_command()  # 현재 위치 재전송
        self.status_label.config(text="Stopped", foreground='red')
    
    def on_closing(self):
        """윈도우 닫기"""
        self.get_logger().info("GUI Controller shutting down...")
        self.root.quit()
        self.root.destroy()
    
    def run_gui(self):
        """GUI 실행"""
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    
    # GUI 컨트롤러 생성
    controller = RobotGUIController()
    
    # ROS 스핀을 별도 스레드에서 실행
    def ros_spin():
        try:
            rclpy.spin(controller)
        except Exception:
            pass
    
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    
    try:
        # GUI 실행 (메인 스레드)
        controller.run_gui()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
