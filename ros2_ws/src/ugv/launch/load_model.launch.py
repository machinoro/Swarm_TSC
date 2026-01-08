import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("ugv")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    urdf_path = os.path.join(pkg, "urdf", "ugv.urdf.xacro")
    robot_desc = xacro.process_file(urdf_path).toxml()

    resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=[os.path.join(pkg, "..")]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc, "use_sim_time": True}],
        output="screen",
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "ugv",
            "-string",
            robot_desc,
            "-z",
            "0.1",
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/model/ugv/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/model/ugv/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
        output="screen",
    )

    return LaunchDescription(
        [resource_path, robot_state_publisher, gazebo_sim, spawn_robot, bridge]
    )
