import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_uav = get_package_share_directory("uav")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    urdf_path = os.path.join(pkg_uav, "urdf", "tarot.urdf.xacro")

    resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH", value=[os.path.join(pkg_uav, "..")]
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-r empty.sdf -v 4"}.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "tarot",
            "-file",
            urdf_path,
            "-z",
            "1.0",
        ],
        output="screen",
    )

    return LaunchDescription([resource_path, gazebo_sim, spawn_robot])
