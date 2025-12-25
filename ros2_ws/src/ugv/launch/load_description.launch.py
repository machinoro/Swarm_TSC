import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("ugv")

    robot_namespace_arg = DeclareLaunchArgument(
        "robot_namespace", default_value="/", description="Namespace for the robot"
    )

    xacro_file = os.path.join(pkg_share, "urdf", "ugv.urdf.xacro")

    robot_description_config = xacro.process_file(xacro_file)
    robot_description_xml = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=LaunchConfiguration("robot_namespace"),
        parameters=[{"robot_description": robot_description_xml}],
    )

    return LaunchDescription([robot_namespace_arg, robot_state_publisher_node])
