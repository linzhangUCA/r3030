from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from math import pi


def generate_launch_description():
    package_path = get_package_share_path("r3030_control")

    sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        choices=["true", "false"],
        description="Flag to enable use simulation time",
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_path / "launch/rplidar.launch.py")
        ),
    )

    footprint_static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "-0.06",
            "--yaw",
            "0",
            "--pitch",
            "0",
            "--roll",
            "0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "base_footprint",
        ],
    )

    lidar_static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "-0.16",
            "--y",
            "0",
            "--z",
            "0.12",
            "--yaw",
            str(pi),
            "--pitch",
            "0",
            "--roll",
            "0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "lidar_link",
        ],
    )

    driver_node = Node(package="r3030_control", executable="driver_interface")

    return LaunchDescription(
        [
            sim_time_arg,
            driver_node,
            rplidar_launch,
            footprint_static_tf_node,
            lidar_static_tf_node,
        ]
    )
