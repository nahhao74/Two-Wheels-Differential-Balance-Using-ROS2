import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('2wheels')

    gazebo_effort_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'gz_effor.launch.py')
        )
    )

    lqr_controller = Node(
        package='2wheels',
        executable='lqr_controller.py',
        name='self_balance_lqr_controller',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            
            # Kích thước thật từ URDF của bạn
            {'wheel_radius': 0.085},
            
            # Giới hạn vật lý từ URDF
            {'torque_limit': 12.0},
            {'wheel_velocity_limit': 7.27},
            {'overspeed_brake_gain': 4.0},
            {'torque_slew_rate_limit': 30.0},
            {'max_tilt_rad': 0.7},
            
            # Ma trận Q, R 
            {'q_diag': [1.0, 2000.0, 1.0, 100.0]}, 
            
            # 3. Tăng R để phạt việc dùng lực mạnh, ép hệ thống tính toán ra mô-men xoắn êm hơn
            {'r_diag': [20.0]},
            
            # QUAN TRỌNG: Đảo chiều lực (torque_sign) để xe chạy đúng hướng ngã
            {'pitch_sign': 1.0},
            {'torque_sign': -1.0}, 
            {'invert_left_wheel': False},
            {'invert_right_wheel': False},
        ],
    )

    return LaunchDescription([
        gazebo_effort_launch,
        TimerAction(period=10.0, actions=[lqr_controller]),
    ])