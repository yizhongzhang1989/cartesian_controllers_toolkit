"""Launch the canonical-URDF source: aux_frame_manager + pre-activate guard.

This is the building block that makes the FZI Cartesian controllers read their
URDF from a single latched topic. Run it AFTER the basic robot bringup (which
publishes the manufacturer URDF on ``/robot_description``) and BEFORE / alongside
the Cartesian controllers (which must be configured with
``urdf_from_topic:=true`` and ``robot_description_topic:=/cartesian/robot_description``).

    ros2 launch aux_frame_manager cartesian_urdf_source.launch.py \
        end_effector_link:=compliance_link \
        aux_frames:='ft_sensor_link:link_6; compliance_link:ft_sensor_link'

The guard latches ``/cartesian/robot_description_ready`` (Bool) once the
configured endpoint/reference frames are present and in-chain; it prints an
actionable error otherwise.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("base_urdf_topic", default_value="/robot_description"),
        DeclareLaunchArgument("output_topic",
                              default_value="/cartesian/robot_description"),
        DeclareLaunchArgument("update_robot_state_publisher", default_value="true"),
        DeclareLaunchArgument("robot_state_publisher_name",
                              default_value="robot_state_publisher"),
        DeclareLaunchArgument("config_file", default_value=""),
        DeclareLaunchArgument("aux_frames_section", default_value=""),
        # Direct-argument frames: rcl-safe compact specs
        # 'name:parent[:x,y,z[:r,p,yw]]' separated by ';'. Overrides/extends the
        # config-file frames. Empty -> use only the config file.
        DeclareLaunchArgument("aux_frames", default_value=""),
        # Guard: the FZI endpoint + reference frames to verify are in-chain.
        DeclareLaunchArgument("robot_base_link", default_value="base_link"),
        DeclareLaunchArgument("end_effector_link", default_value=""),
        # NOTE: default is ['']  (one empty string), NOT []  -- an empty list
        # is rejected by the ROS 2 parameter parser ("Expected a non-empty
        # sequence"). The guard filters out the empty entry, so [''] behaves
        # like "no required frames".
        DeclareLaunchArgument("required_frames", default_value="['']"),
        DeclareLaunchArgument("enable_guard", default_value="true"),
        # Optional dashboard (3D view + live editor); started when set.
        DeclareLaunchArgument("dashboard_port", default_value=""),
    ]

    manager = Node(
        package="aux_frame_manager",
        executable="aux_frame_manager",
        name="aux_frame_manager",
        output="screen",
        parameters=[{
            "base_urdf_topic": LaunchConfiguration("base_urdf_topic"),
            "output_topic": LaunchConfiguration("output_topic"),
            "update_robot_state_publisher":
                LaunchConfiguration("update_robot_state_publisher"),
            "robot_state_publisher_name":
                LaunchConfiguration("robot_state_publisher_name"),
            "config_file": LaunchConfiguration("config_file"),
            "aux_frames_section": LaunchConfiguration("aux_frames_section"),
            "aux_frames": LaunchConfiguration("aux_frames"),
        }],
    )

    guard = Node(
        package="aux_frame_manager",
        executable="aux_frame_guard",
        name="aux_frame_guard",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_guard")),
        parameters=[{
            "robot_description_topic": LaunchConfiguration("output_topic"),
            "robot_base_link": LaunchConfiguration("robot_base_link"),
            "end_effector_link": LaunchConfiguration("end_effector_link"),
            "required_frames": LaunchConfiguration("required_frames"),
        }],
    )

    dashboard = Node(
        package="aux_frame_manager",
        executable="aux_frame_dashboard",
        name="aux_frame_dashboard",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("dashboard_port"),
                              "' != ''"])),
        parameters=[{
            "port": LaunchConfiguration("dashboard_port"),
            "manager_ns": "/aux_frame_manager",
            "base_frame": LaunchConfiguration("robot_base_link"),
            "canonical_topic": LaunchConfiguration("output_topic"),
            "base_urdf_topic": LaunchConfiguration("base_urdf_topic"),
        }],
    )

    return LaunchDescription(args + [manager, guard, dashboard])
