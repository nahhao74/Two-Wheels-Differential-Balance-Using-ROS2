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


class LqrController(Node):
    def __init__(self):
        super().__init__('self_balance_lqr_controller')

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
        self.declare_parameter('torque_slew_rate_limit', 60.0)
        self.declare_parameter('max_tilt_rad', 0.7)
        
        self.declare_parameter('q_diag', [190.0, 16130.0, 100.0, 14.0])
        self.declare_parameter('r_diag', [1.0]) 
        
        self.declare_parameter('pitch_sign', 1.0)
        self.declare_parameter('torque_sign', -1.0) # Đã đảo dấu
        self.declare_parameter('invert_left_wheel', False)
        self.declare_parameter('invert_right_wheel', False)

        self.left_joint = self.get_parameter('left_joint').value
        self.right_joint = self.get_parameter('right_joint').value
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.torque_limit = float(self.get_parameter('torque_limit').value)
        self.wheel_velocity_limit = float(self.get_parameter('wheel_velocity_limit').value)
        self.overspeed_brake_gain = float(self.get_parameter('overspeed_brake_gain').value)
        self.torque_slew_rate_limit = float(self.get_parameter('torque_slew_rate_limit').value)
        self.max_tilt_rad = float(self.get_parameter('max_tilt_rad').value)
        self.pitch_sign = float(self.get_parameter('pitch_sign').value)
        self.torque_sign = float(self.get_parameter('torque_sign').value)
        self.invert_left_wheel = bool(self.get_parameter('invert_left_wheel').value)
        self.invert_right_wheel = bool(self.get_parameter('invert_right_wheel').value)

        q_diag = list(self.get_parameter('q_diag').value)
        r_diag = list(self.get_parameter('r_diag').value)
        
        self.k_gain = compute_lqr_gain(q_diag, r_diag)
        self.get_logger().info(f'LQR K gain:\n{self.k_gain}')

        self.imu_msg = None
        self.joint_msg = None
        
        self.last_time = None
        self.last_torque = np.zeros(2)

        self.command_pub = self.create_publisher(
            Float64MultiArray,
            self.get_parameter('command_topic').value,
            10,
        )
        self.create_subscription(Imu, self.get_parameter('imu_topic').value, self.imu_callback, 10)
        self.create_subscription(JointState, self.get_parameter('joint_states_topic').value, self.joint_state_callback, 10)

        control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.timer = self.create_timer(1.0 / control_rate_hz, self.control_step)

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

    def publish_torque(self, left_torque, right_torque):
        msg = Float64MultiArray()
        # Phải/Trái tùy thuộc vào thứ tự controller.yaml, ở đây ta cấp chung 1 effort
        msg.data = [float(right_torque), float(left_torque)] 
        self.command_pub.publish(msg)

    def limit_wheel_speed(self, torque, velocity):
        if velocity > self.wheel_velocity_limit:
            brake = -self.overspeed_brake_gain * (velocity - self.wheel_velocity_limit)
            return min(torque, brake)
        if velocity < -self.wheel_velocity_limit:
            brake = -self.overspeed_brake_gain * (velocity + self.wheel_velocity_limit)
            return max(torque, brake)
        return torque

    def limit_torque_slew_rate(self, torque, dt):
        if dt <= 0.0:
            return torque
        max_delta = self.torque_slew_rate_limit * dt
        delta = np.clip(torque - self.last_torque, -max_delta, max_delta)
        return self.last_torque + delta

    def control_step(self):
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            self.publish_torque(0.0, 0.0)
            return

        dt = max((now - self.last_time).nanoseconds * 1e-9, 0.0)
        self.last_time = now

        if self.imu_msg is None or self.joint_msg is None:
            self.publish_torque(0.0, 0.0)
            return

        left_pos, left_vel = self.get_joint_state(self.left_joint)
        right_pos, right_vel = self.get_joint_state(self.right_joint)
        if left_pos is None or right_pos is None:
            self.publish_torque(0.0, 0.0)
            return

        if self.invert_left_wheel:
            left_pos, left_vel = -left_pos, -left_vel
        if self.invert_right_wheel:
            right_pos, right_vel = -right_pos, -right_vel

        # IMU: Lấy góc Pitch và Tốc độ góc
        orientation = self.imu_msg.orientation
        _, pitch, _ = quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        pitch *= self.pitch_sign
        pitch_rate = self.pitch_sign * self.imu_msg.angular_velocity.y

        # Encoder: Tính vị trí x (m) và x_dot (m/s)
        robot_x = 0.5 * (left_pos + right_pos) * self.wheel_radius
        robot_v = 0.5 * (left_vel + right_vel) * self.wheel_radius

        # ==========================================
        # MA TRẬN TRẠNG THÁI (Thực tế và Mục tiêu)
        # ==========================================
        x_state = np.array([robot_x, pitch, robot_v, pitch_rate])
        
        # Đã ép cứng mục tiêu về 0 0 0 0
        x_target = np.array([0.0, 0.0, 0.0, 0.0]) 
        
        error = x_state - x_target

        if abs(pitch) > self.max_tilt_rad:
            self.publish_torque(0.0, 0.0)
            return

        # Tính toán u. Do torque_sign đã đổi thành -1.0 trong launch, dấu u sẽ được lật lại.
        u = -self.torque_sign * (self.k_gain @ error)
        base_torque = u[0]

        torque = np.array([base_torque, base_torque])
        
        torque = np.clip(torque, -self.torque_limit, self.torque_limit)
        torque[0] = self.limit_wheel_speed(torque[0], right_vel)
        torque[1] = self.limit_wheel_speed(torque[1], left_vel)
        torque = self.limit_torque_slew_rate(torque, dt)
        torque = np.clip(torque, -self.torque_limit, self.torque_limit)
        
        self.last_torque = torque
        self.publish_torque(torque[0], torque[1])

def main():
    rclpy.init()
    node = LqrController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_torque(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()