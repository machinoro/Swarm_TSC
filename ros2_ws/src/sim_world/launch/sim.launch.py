import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_uav = get_package_share_directory("uav")
    pkg_ugv = get_package_share_directory("ugv")

    uav_xacro_file = os.path.join(pkg_uav, "urdf", "tarot.urdf.xacro")
    uav_xml = xacro.process_file(uav_xacro_file).toxml()

    ugv_xacro_file = os.path.join(pkg_ugv, "urdf", "ugv.urdf.xacro")
    ugv_xml = xacro.process_file(ugv_xacro_file).toxml()

    uav_parent_dir = os.path.dirname(pkg_uav)
    ugv_parent_dir = os.path.dirname(pkg_ugv)

    resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[
            uav_parent_dir,
            ":",
            ugv_parent_dir,
            ":",
            os.path.join(pkg_uav, "share"),
            ":",
            os.path.join(pkg_ugv, "share"),
        ],
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-r empty.sdf -v 4"}.items(),
    )

    rsp_ugv = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": ugv_xml, "use_sim_time": True}],
    )

    spawn_uav = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "tarot", "-string", uav_xml, "-z", "1.0"],
        output="screen",
    )

    spawn_ugv = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "ugv",
            "-topic",
            "robot_description",
            "-string",
            ugv_xml,
            "-x",
            "2.0",
            "-z",
            "1.0",
        ],
        output="screen",
    )

    return LaunchDescription([resource_path, gazebo_sim, rsp_ugv, spawn_ugv, spawn_uav])
