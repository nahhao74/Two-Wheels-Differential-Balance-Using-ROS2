from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    robot_path = get_package_share_directory('2wheels')
    urdf_file = os.path.join(robot_path, 'urdf', 'simplied.urdf')
    
    pkg_share = FindPackageShare('2wheels')

    world_file = PathJoinSubstitution([
        pkg_share,
        'worlds',
        'empty.sdf'
    ])

    model_path = os.path.expanduser(
        '~/Robot_sim_ws/2wheels/models'
    )

    return LaunchDescription([

        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=model_path
        ),

        ExecuteProcess(
            cmd=['gz', 'sim', world_file],
            output='screen'
        ),

         # Spawn robot
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-name', 'my_robot',
                '-file', urdf_file,
                '-x', '0',
                '-y', '0',
                '-z', '0.1'
            ],
            output='screen')

    ])
