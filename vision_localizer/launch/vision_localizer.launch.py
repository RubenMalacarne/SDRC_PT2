from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    config_path = PathJoinSubstitution(
        [
            FindPackageShare("vision_localizer"),
            "config",
            "object_localizer_params.yaml"
        ]
    )

    object_localizer = Node(
        package='vision_localizer',
        executable='object_localizer',
        name='object_localizer',
        parameters=[config_path],
        output='screen'
    )

    return LaunchDescription([
        object_localizer
    ])
