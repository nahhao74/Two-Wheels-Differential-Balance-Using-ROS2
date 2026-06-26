#!/usr/bin/python3
import math
import numpy as np
from scipy.linalg import solve_continuous_are

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray


def quaternion_to_euler(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def compute_lqr_gain(q_diag, r_diag):
    a_matrix = np.array([
        [0.0, 0.0, 1.0000, 0.0],
        [0.0, 0.0, 0.0, 1.0000],
        [0.0, -5.4600, 0.0, 0.0],
        [0.0, 55.3603, 0.0, 0.0]
    ], dtype=float)

    b_matrix = np.array([
        [0.0],
        [0.0],
        [-1.6912],
        [8.7266]
    ], dtype=float)

    q_matrix = np.diag(q_diag)
    r_matrix = np.diag(r_diag)

    s_matrix = solve_continuous_are(a_matrix, b_matrix, q_matrix, r_matrix)
    return np.linalg.solve(r_matrix, b_matrix.T @ s_matrix)


class LqrPidController(Node):
    def __init__(self):
        super().__init__('self_balance_controller')

        self.declare_parameter('imu_topic', '/base_imu')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('command_topic', '/self_balance_controller/commands')
        self.declare_parameter('left_joint', 'left_wheel_joint')   
        self.declare_parameter('right_joint', 'right_wheel_joint')
        
        self.declare_parameter('control_rate_hz', 100.0)
        self.declare_parameter('wheel_radius', 0.085) 
        self.declare_parameter('torque_limit', 12.0) 
        self.declare_parameter('wheel_velocity_limit', 7.27) 
        self.declare_parameter('overspeed_brake_gain', 4.0)
        self.declare_parameter('torque_slew_rate_limit', 30.0)
        self.declare_parameter('max_tilt_rad', 0.7)
        
        # --- LQR ---
        self.declare_parameter('q_diag', [1.0, 2000.0, 1.0, 100.0])
        self.declare_parameter('r_diag', [20.0]) 
        
        # --- PID ---
        self.declare_parameter('yaw_kp', 2.0)
        self.declare_parameter('yaw_ki', 0.0)
        self.declare_parameter('yaw_kd', 0.5)
        
        self.declare_parameter('pitch_sign', 1.0)
        self.declare_parameter('torque_sign', -1.0)
        self.declare_parameter('yaw_sign', -1.0) 
        self.declare_parameter('invert_left_wheel', False)
        self.declare_parameter('invert_right_wheel', False)

        # Đọc tham số
        self.left_joint = self.get_parameter('left_joint').value
        self.right_joint = self.get_parameter('right_joint').value
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.torque_limit = float(self.get_parameter('torque_limit').value)
        self.wheel_velocity_limit = float(self.get_parameter('wheel_velocity_limit').value)
        self.overspeed_brake_gain = float(self.get_parameter('overspeed_brake_gain').value)
        self.torque_slew_rate_limit = float(self.get_parameter('torque_slew_rate_limit').value)
        self.max_tilt_rad = float(self.get_parameter('max_tilt_rad').value)
        
        self.yaw_kp = float(self.get_parameter('yaw_kp').value)
        self.yaw_ki = float(self.get_parameter('yaw_ki').value)
        self.yaw_kd = float(self.get_parameter('yaw_kd').value)

        self.pitch_sign = float(self.get_parameter('pitch_sign').value)
        self.torque_sign = float(self.get_parameter('torque_sign').value)
        self.yaw_sign = float(self.get_parameter('yaw_sign').value)
        self.invert_left_wheel = bool(self.get_parameter('invert_left_wheel').value)
        self.invert_right_wheel = bool(self.get_parameter('invert_right_wheel').value)

        # Tính LQR Gain
        q_diag = list(self.get_parameter('q_diag').value)
        r_diag = list(self.get_parameter('r_diag').value)
        self.k_gain = compute_lqr_gain(q_diag, r_diag)

        # Biến trạng thái nội bộ
        self.imu_msg = None
        self.joint_msg = None
        self.last_time = None
        self.last_left_torque = 0.0
        self.last_right_torque = 0.0
        self.is_yaw_initialized = False
        
        # Biến mục tiêu mặc định
        self.target_x = 0.0
        self.target_v = 0.0
        self.target_yaw = 0.0
        self.target_yaw_rate = 0.0
        self.yaw_integral = 0.0
        self.gui_connected = False

        # Các Pub/Sub
        self.command_pub = self.create_publisher(Float64MultiArray, self.get_parameter('command_topic').value, 10)
        self.create_subscription(Imu, self.get_parameter('imu_topic').value, self.imu_callback, 10)
        self.create_subscription(JointState, self.get_parameter('joint_states_topic').value, self.joint_state_callback, 10)
        
        # Khởi tạo Subscriber lắng nghe GUI
        self.create_subscription(Float64MultiArray, '/gui_targets', self.gui_callback, 10)

        control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.timer = self.create_timer(1.0 / control_rate_hz, self.control_step)

    def gui_callback(self, msg):
        """Hàm nhận lệnh mục tiêu từ bảng điều khiển GUI"""
        if len(msg.data) == 4:
            self.target_x = msg.data[0]
            self.target_v = msg.data[1]
            self.target_yaw = msg.data[2]
            self.target_yaw_rate = msg.data[3]
            self.gui_connected = True

    def imu_callback(self, msg):
        self.imu_msg = msg

    def joint_state_callback(self, msg):
        self.joint_msg = msg

    def get_joint_state(self, joint_name):
        if self.joint_msg is None or joint_name not in self.joint_msg.name:
            return None, None
        index = self.joint_msg.name.index(joint_name)
        position = self.joint_msg.position[index] if index < len(self.joint_msg.position) else 0.0
        velocity = self.joint_msg.velocity[index] if index < len(self.joint_msg.velocity) else 0.0
        return position, velocity

    def limit_wheel_speed(self, torque, velocity):
        if velocity > self.wheel_velocity_limit:
            brake = -self.overspeed_brake_gain * (velocity - self.wheel_velocity_limit)
            return min(torque, brake)
        if velocity < -self.wheel_velocity_limit:
            brake = -self.overspeed_brake_gain * (velocity + self.wheel_velocity_limit)
            return max(torque, brake)
        return torque

    def wrap_to_pi(self, angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def control_step(self):
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return

        dt = max((now - self.last_time).nanoseconds * 1e-9, 0.001)
        self.last_time = now

        if self.imu_msg is None or self.joint_msg is None:
            return

        left_pos, left_vel = self.get_joint_state(self.left_joint)
        right_pos, right_vel = self.get_joint_state(self.right_joint)
        if left_pos is None or right_pos is None:
            return

        if self.invert_left_wheel:
            left_pos, left_vel = -left_pos, -left_vel
        if self.invert_right_wheel:
            right_pos, right_vel = -right_pos, -right_vel

        # 1. ĐỌC CẢM BIẾN
        orientation = self.imu_msg.orientation
        _, pitch, yaw = quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        pitch *= self.pitch_sign
        pitch_rate = self.pitch_sign * self.imu_msg.angular_velocity.y
        yaw_rate = self.imu_msg.angular_velocity.z

        # Lưu góc khởi điểm làm mục tiêu nếu GUI chưa gửi lệnh
        if not self.is_yaw_initialized:
            self.target_yaw = yaw
            self.is_yaw_initialized = True

        robot_x = 0.5 * (left_pos + right_pos) * self.wheel_radius
        robot_v = 0.5 * (left_vel + right_vel) * self.wheel_radius

        # An toàn ngã
        if abs(pitch) > self.max_tilt_rad:
            msg = Float64MultiArray()
            msg.data = [0.0, 0.0]
            self.command_pub.publish(msg)
            self.yaw_integral = 0.0 
            return

        # ==========================================
        # 2. KHÂU LQR (Mục tiêu = GUI Targets)
        # ==========================================
        x_state = np.array([robot_x, pitch, robot_v, pitch_rate])
        
        # Nếu chưa bật GUI, mục tiêu mặc định là 0. 
        # Nếu đã bật, đưa dữ liệu GUI vào ma trận mục tiêu
        if self.gui_connected:
            x_target = np.array([self.target_x, 0.0, self.target_v, 0.0])
        else:
            x_target = np.array([0.0, 0.0, 0.0, 0.0])
            
        error_lqr = x_state - x_target
        u_lqr = -self.torque_sign * (self.k_gain @ error_lqr)
        u_balance = u_lqr[0]

        # ==========================================
        # 3. KHÂU PID YAW (Mục tiêu = GUI Targets)
        # ==========================================
        yaw_error = self.wrap_to_pi(self.target_yaw - yaw)
        
        self.yaw_integral += yaw_error * dt
        self.yaw_integral = np.clip(self.yaw_integral, -5.0, 5.0) 
        
        # Dùng target_yaw_rate - yaw_rate để xe xoay mượt mà theo vận tốc yêu cầu
        yaw_rate_error = self.target_yaw_rate - yaw_rate
        u_yaw = self.yaw_kp * yaw_error + self.yaw_ki * self.yaw_integral + self.yaw_kd * yaw_rate_error
        u_yaw *= self.yaw_sign

        # ==========================================
        # 4. TRỘN TÍN HIỆU
        # ==========================================
        right_torque = u_balance + u_yaw
        left_torque = u_balance - u_yaw
        
        right_torque = np.clip(right_torque, -self.torque_limit, self.torque_limit)
        left_torque = np.clip(left_torque, -self.torque_limit, self.torque_limit)
        
        right_torque = self.limit_wheel_speed(right_torque, right_vel)
        left_torque = self.limit_wheel_speed(left_torque, left_vel)
        
        max_delta = self.torque_slew_rate_limit * dt
        right_torque = np.clip(right_torque, self.last_right_torque - max_delta, self.last_right_torque + max_delta)
        left_torque = np.clip(left_torque, self.last_left_torque - max_delta, self.last_left_torque + max_delta)
        
        self.last_right_torque = right_torque
        self.last_left_torque = left_torque

        msg = Float64MultiArray()
        msg.data = [float(right_torque), float(left_torque)]
        self.command_pub.publish(msg)

def main():
    rclpy.init()
    node = LqrPidController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0]
        node.command_pub.publish(msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()