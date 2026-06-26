#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import tkinter as tk
from tkinter import ttk
import threading

class RobotGuiNode(Node):
    def __init__(self):
        super().__init__('robot_gui_node')
        # Gửi dữ liệu xuống topic '/gui_targets'
        self.publisher_ = self.create_publisher(Float64MultiArray, '/gui_targets', 10)
        
        # 1. Đích đến mong muốn (Lấy từ thanh trượt GUI)
        self.desired_targets = [0.0, 0.0, 0.0, 0.0] # [x, v, yaw, yaw_rate]
        
        # 2. Giá trị thực tế đang gửi (Sẽ bám theo đích đến từ từ)
        self.current_targets = [0.0, 0.0, 0.0, 0.0]
        
        # 3. Tốc độ thay đổi tối đa mỗi chu kỳ 0.1 giây (Slew rate limits)
        # - x: tối đa 0.05m/0.1s = 0.5 m/s
        # - v: tối đa 0.1m/s / 0.1s = 1.0 m/s^2 (Gia tốc)
        # - yaw: tối đa 0.05 rad / 0.1s = 0.5 rad/s
        # - yaw_rate: tối đa 0.2 rad/s / 0.1s = 2.0 rad/s^2
        self.max_steps = [0.05, 0.1, 0.05, 0.2]
        
        # Publish liên tục với tần số 10Hz (dt = 0.1s)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Biến tham chiếu UI để cập nhật text (được gán từ class App)
        self.ui_labels = []

    def timer_callback(self):
        # Tính toán giá trị trung gian (Làm mềm tín hiệu)
        for i in range(4):
            error = self.desired_targets[i] - self.current_targets[i]
            
            if error > self.max_steps[i]:
                self.current_targets[i] += self.max_steps[i] # Tăng từ từ
            elif error < -self.max_steps[i]:
                self.current_targets[i] -= self.max_steps[i] # Giảm từ từ
            else:
                self.current_targets[i] = self.desired_targets[i] # Đã tới đích
                
        # Đóng gói và gửi tín hiệu
        msg = Float64MultiArray()
        msg.data = self.current_targets
        self.publisher_.publish(msg)
        
        # Cập nhật hiển thị lên GUI (để user thấy số đang chạy)
        if len(self.ui_labels) == 4:
            for i in range(4):
                # Sử dụng after để gọi update UI từ luồng chính an toàn
                self.ui_labels[i].after(0, self.update_label, i, self.current_targets[i])

    def update_label(self, index, value):
        self.ui_labels[index].config(text=f"Đang xuất: {value:.3f}")

class App(tk.Tk):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.title("Bảng điều khiển Tịnh Tiến (Có chống giật)")
        self.geometry("450x420")
        
        # Cấu hình lưới
        self.columnconfigure(1, weight=1)

        # Định nghĩa các thông số
        self.params = [
            {"name": "Vị trí xe (m)", "min": -10.0, "max": 10.0, "default": 0.0, "step": 0.1},
            {"name": "Vận tốc xe (m/s)", "min": -5.0, "max": 5.0, "default": 0.0, "step": 0.1},
            {"name": "Góc Yaw (rad)", "min": -3.14, "max": 3.14, "default": 0.0, "step": 0.01},
            {"name": "Vận tốc Yaw (rad/s)", "min": -5.0, "max": 5.0, "default": 0.0, "step": 0.1}
        ]
        
        self.vars = []

        # Tạo giao diện
        for i, param in enumerate(self.params):
            # Tiêu đề
            ttk.Label(self, text=param["name"], font=('Arial', 10, 'bold')).grid(
                row=i*3, column=0, columnspan=2, pady=(15,0), sticky="w", padx=10)
            
            # Label hiển thị giá trị thật đang chạy
            lbl_current = ttk.Label(self, text="Đang xuất: 0.000", foreground="blue")
            lbl_current.grid(row=i*3, column=2, pady=(15,0), sticky="e", padx=10)
            self.ros_node.ui_labels.append(lbl_current)
            
            # Biến liên kết dữ liệu
            var = tk.DoubleVar(value=param["default"])
            self.vars.append(var)
            
            # Khung nhập số (Spinbox)
            spin = ttk.Spinbox(self, from_=param["min"], to=param["max"], increment=param["step"], 
                               textvariable=var, width=8, command=self.update_ros)
            spin.grid(row=i*3+1, column=0, padx=10, sticky="w")
            
            # Thanh trượt (Scale)
            scale = ttk.Scale(self, from_=param["min"], to=param["max"], variable=var, 
                              command=lambda event, v=var: self.on_scale_move(v))
            scale.grid(row=i*3+1, column=1, columnspan=2, sticky="ew", padx=10)
            
            var.trace_add("write", lambda *args: self.update_ros())

        # Nút Reset
        reset_btn = ttk.Button(self, text="Dừng khẩn cấp / Về 0", command=self.reset_values)
        reset_btn.grid(row=15, column=0, columnspan=3, pady=20)

    def on_scale_move(self, var):
        # Làm tròn giá trị Scale 
        var.set(round(var.get(), 2))
        self.update_ros()

    def update_ros(self):
        try:
            # Chỉ cập nhật đích đến, timer_callback sẽ lo việc đi tới đích từ từ
            data = [float(var.get()) for var in self.vars]
            self.ros_node.desired_targets = data
        except ValueError:
            pass

    def reset_values(self):
        # Khi bấm reset, đích về 0 ngay lập tức, xe sẽ phanh lại từ từ theo Slew rate
        for var in self.vars:
            var.set(0.0)
        self.update_ros()

def ros_spin_thread(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    
    ros_node = RobotGuiNode()
    
    # Chạy ROS 2 spin trong một luồng riêng
    spin_thread = threading.Thread(target=ros_spin_thread, args=(ros_node,), daemon=True)
    spin_thread.start()
    
    # Chạy giao diện Tkinter
    app = App(ros_node)
    app.mainloop()
    
    # Dọn dẹp
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()