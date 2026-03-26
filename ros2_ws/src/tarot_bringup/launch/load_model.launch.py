import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_description = get_package_share_directory("tarot_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_bringup = get_package_share_directory("tarot_bringup")

    urdf_path = os.path.join(pkg_description, "urdf", "tarot.urdf.xacro")
    rviz_config = os.path.join(pkg_bringup, "config", "tarot.rviz")

    resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=[os.path.join(pkg_description, "..")]
    )

    robot_desc = xacro.process_file(urdf_path).toxml()

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/os1/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked"],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "tarot",
            "-string",
            robot_desc,
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen",
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription(
        [
            resource_path,
            gazebo_sim,
            spawn_robot,
            ros_gz_bridge,
            robot_state_publisher,
            rviz2,
            joint_state_publisher,
        ]
    )
