import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = get_package_share_directory('coppelia_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'ur5e.urdf')

    # Legge il file URDF
    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("coppelia_controller"),
            "config",
            "ur5e_control.yaml",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("coppelia_controller"), "config", "rviz_config.rviz"]
    )

    # Nodo robot_state_publisher per pubblicare il robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{"robot_description": robot_description}]
    )

    # Avvia il nodo controller manager (ros2_control_node) e passa il parametro robot_description
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="screen",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    # Spawn del joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]
    )

    # Spawn del joint_trajectory_position_controller
    arm_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["arm_trajectory_controller", "--controller-manager", "/controller_manager"]
    )
    
    # Spawn del controller del gripper
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"]
    )


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file]
    )

    return LaunchDescription([
        robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        arm_trajectory_controller_spawner,
        gripper_controller_spawner,
       # rviz_node
    ])
