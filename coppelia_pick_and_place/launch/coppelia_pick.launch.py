import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )

    is_sim = LaunchConfiguration("is_sim")

    # Costruisci configurazione MoveIt
    moveit_config = (
        MoveItConfigsBuilder(robot_name="arm_manipulator", package_name="coppelia_moveit")
        .robot_description()
        .robot_description_semantic()
        .trajectory_execution()
        .planning_pipelines()
        .to_moveit_configs()
    )
    
    test_action_server_node = Node(
        package="coppelia_pick_and_place",
        executable="test_action_server_node",
        parameters=[    
            moveit_config.to_dict(),
            {"use_sim_time": is_sim}]
    )
    
    motion_action_server = Node(
        package="coppelia_pick_and_place",
        executable="motion_action_server_node",
        parameters=[    
            moveit_config.to_dict(),
            {"use_sim_time": is_sim}]
    )
    return LaunchDescription([
        is_sim_arg,
        #test_action_server_node,
        motion_action_server
    
    ])