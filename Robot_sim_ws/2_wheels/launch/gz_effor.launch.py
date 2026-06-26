import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable

from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('2wheels')

    urdf_file = os.path.join(pkg, 'urdf', 'simplied.urdf')
    world_file = os.path.join(pkg, 'worlds', 'empty.sdf')
    bridge_config = os.path.join(pkg, 'config', 'bridge_config.yaml')
    controller_config = os.path.join(pkg, 'config', 'controller.yaml')
    models_dir = os.path.join(pkg, 'models')

    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Meshes
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{os.path.dirname(pkg)}:{models_dir}"
    )


    
    # -------------------------
    # Controller YAML passed     # to gz_ros2_control
    # -------------------------
    set_ros_args = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_ARGS',
        value=f'--ros-args --params-file {controller_config}'
    )

    # -------------------------
    # Robot State Publisher
    # -------------------------

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ],
        output='screen'
    )

    # -------------------------
    # Start Gazebo
    # -------------------------

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # -------------------------
    # Spawn robot
    # -------------------------

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'omni_base',
            '-x','0.0',
            '-y','0.0',
            '-z', '0.1',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen'
    )

    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_robot]
    )

    # -------------------------
    # ROS <-> Gazebo Bridge
    # Uses bridge_config.yaml
    # -------------------------

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    delayed_bridge = TimerAction(
        period=5.0,
        actions=[bridge]
    )

    # -------------------------
    # ros2_control Controllers
    # -------------------------

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    self_balance_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'self_balance_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )


    delayed_controllers = TimerAction(
        period=8.0,
        actions=[
            joint_state_broadcaster,
            self_balance_controller,
        ]
    )


    # -------------------------
    # Launch everything
    # -------------------------

    return LaunchDescription([
        set_gz_resource_path,
        set_ros_args,
        robot_state_publisher,
        gz_sim,
        delayed_spawn,
        delayed_bridge,
        delayed_controllers,
    ])