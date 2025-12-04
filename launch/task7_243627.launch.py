import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

package_name = "ias0220_243627"


def generate_launch_description():
    package_path = os.path.join(get_package_share_directory(package_name))

    xacro_file = os.path.join(package_path, "urdf",
                              "differential_robot_simu_task4_part2.urdf")
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)

    config = os.path.join(
        package_path,
        'config',
        'simple_control_v2.yaml'
    )

    control_node = Node(
        package=package_name,
        executable="control_node",
        name="controller",
        output="screen",
        parameters=[config],
    )

    rvizconfig = LaunchConfiguration(
        "rvizconfig",
        default=os.path.join(
            get_package_share_directory(package_name),
            "config",
            "differential_robot_rviz_task7.rviz",
        ),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output="screen",
        arguments=["--display-config", rvizconfig],
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("setup_gazebo_ias0220"),
                "launch",
                "gazebo.launch.py"
            )
        ),
        launch_arguments={
            "xacro_file": xacro_file
        }.items(),
    )

    static_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_broadcaster",
        output="screen",
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    return LaunchDescription(
        [
            rviz_node,
            static_transform,
            control_node,
            gazebo_launch,
        ]
    )