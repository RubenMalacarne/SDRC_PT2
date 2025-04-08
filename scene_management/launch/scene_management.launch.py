from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )

    is_sim = LaunchConfiguration("is_sim")

    moveit_config = (
        MoveItConfigsBuilder("ur5e", package_name="coppelia_moveit")
        .robot_description(file_path="config/ur5e_with_robotiq85.urdf.xacro")
        .robot_description_semantic(file_path="config/ur5e_with_robotiq85.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl","chomp","pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    ### CORREZIONE CHIAVI - MoveItConfigsBuilder.to_moveit_configs(), sembra avre dei problemi, genera delle chiavi sbagliate

    # A) URDF -> "robot_description"
    old_robot_description = moveit_config.robot_description
    if "moveit_config" in old_robot_description:
        corrected_robot_description = {
            "robot_description": old_robot_description["moveit_config"]
        }
    else:
        corrected_robot_description = old_robot_description

    # B) SRDF -> "robot_description_semantic"
    old_semantic = moveit_config.robot_description_semantic
    if "moveit_config_semantic" in old_semantic:
        corrected_robot_description_semantic = {
            "robot_description_semantic": old_semantic["moveit_config_semantic"]
        }
    else:
        corrected_robot_description_semantic = old_semantic

    # C) Kinematics -> "robot_description_kinematics"
    old_kinematics = moveit_config.robot_description_kinematics
    # Se la chiave è "moveit_config_kinematics", la rinominiamo
    if "moveit_config_kinematics" in old_kinematics:
        corrected_kinematics = {
            "robot_description_kinematics": old_kinematics["moveit_config_kinematics"]
        }
    elif "moveit_config" in old_kinematics:
        # Se invece troviamo "moveit_config", la rinominiamo
        corrected_kinematics = {
            "robot_description_kinematics": old_kinematics["moveit_config"]
        }
    else:
        corrected_kinematics = old_kinematics

    # D) Joint Limits -> "robot_description_planning"
    old_joint_limits = moveit_config.joint_limits
    # Se la chiave è "moveit_config_planning", la rinominiamo
    if "moveit_config_planning" in old_joint_limits:
        corrected_joint_limits = {
            "robot_description_planning": old_joint_limits["moveit_config_planning"]
        }
    elif "moveit_config" in old_joint_limits:
        # Se invece troviamo "moveit_config", la rinominiamo
        corrected_joint_limits = {
            "robot_description_planning": old_joint_limits["moveit_config"]
        }
    else:
        corrected_joint_limits = old_joint_limits

    # E) Pianificazione
    old_planning_pipelines = moveit_config.planning_pipelines
    if "moveit_config" in old_planning_pipelines:
        corrected_planning_pipelines = old_planning_pipelines["moveit_config"]
    else:
        corrected_planning_pipelines = old_planning_pipelines

    # F) Trajectory Execution
    corrected_trajectory_execution = moveit_config.trajectory_execution

    #######################################################

    object_spawner_node = Node(
        package="scene_management",
        executable="object_spawner",
        name="object_spawner",
        output="screen",
        parameters=[
            {"use_sim_time": is_sim},
            corrected_robot_description,
            corrected_robot_description_semantic,
            corrected_kinematics,
            corrected_joint_limits,
            corrected_planning_pipelines,
            corrected_trajectory_execution
        ]   
    )

    return LaunchDescription([
        is_sim_arg,
        object_spawner_node
    ])
