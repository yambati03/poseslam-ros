# Launch the Poseslam node with the specified parameters.

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    poseslam_params = PathJoinSubstitution(
        [FindPackageShare("poseslam"), "config", "poseslam_params.yml"]
    )

    poseslam_node = Node(
        package="poseslam",
        executable="poseslam_PoseSlam",
        output="screen",
        parameters=[poseslam_params]
    )

    return LaunchDescription([
        poseslam_node
    ])