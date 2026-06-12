"""Launch one cct_pose_commander instance.

Defaults control the RIGHT arm via its JointTrajectoryController. Override for
the left arm (or for FPC streaming) on the command line, e.g.::

    ros2 launch cct_pose_commander commander.launch.py \
        instance_name:=left \
        controlled_frame:=left_arm_Link7 \
        jtc_controller:=left_arm_joint_trajectory_controller \
        fpc_controller:=left_arm_forward_position_controller \
        joints:="['left_arm_joint1','left_arm_joint2','left_arm_joint3','left_arm_joint4','left_arm_joint5','left_arm_joint6','left_arm_joint7']"

The node starts DISABLED; call its ``~/enable`` service to allow motion.

Pass ``dashboard_port:=<port>`` to also bring up the web dashboard wired to this
commander instance (omit it / leave empty to run headless), e.g.::

    ros2 launch cct_pose_commander commander.launch.py dashboard_port:=8180
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                   PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument("instance_name", default_value="right"),
        DeclareLaunchArgument("controlled_frame", default_value="right_arm_Link7"),
        DeclareLaunchArgument(
            "jtc_controller",
            default_value="right_arm_joint_trajectory_controller"),
        DeclareLaunchArgument(
            "fpc_controller",
            default_value="right_arm_forward_position_controller"),
        DeclareLaunchArgument("command_mode", default_value="jtc"),
        DeclareLaunchArgument("start_enabled", default_value="false"),
        DeclareLaunchArgument("base_frame", default_value=""),
        DeclareLaunchArgument(
            "joints",
            default_value="['right_arm_joint1','right_arm_joint2',"
                          "'right_arm_joint3','right_arm_joint4',"
                          "'right_arm_joint5','right_arm_joint6',"
                          "'right_arm_joint7']"),
        # Empty => headless (no dashboard). Any port => also launch the dashboard
        # wired to this commander instance.
        DeclareLaunchArgument("dashboard_port", default_value=""),
        # TF frame the dashboard captures/jogs in (a concrete frame, not the
        # commander's possibly-empty solve base_frame).
        DeclareLaunchArgument("dashboard_base_frame", default_value="base_link"),
    ]

    node = Node(
        package="cct_pose_commander",
        executable="commander_node",
        name=PythonExpression(
            ["'cct_pose_commander_' + '",
             LaunchConfiguration("instance_name"), "'"]),
        output="screen",
        parameters=[{
            "controlled_frame": LaunchConfiguration("controlled_frame"),
            "jtc_controller": LaunchConfiguration("jtc_controller"),
            "fpc_controller": LaunchConfiguration("fpc_controller"),
            "command_mode": LaunchConfiguration("command_mode"),
            "start_enabled": LaunchConfiguration("start_enabled"),
            "base_frame": LaunchConfiguration("base_frame"),
            "joints": LaunchConfiguration("joints"),
        }],
    )

    # Conditionally include the dashboard when dashboard_port is non-empty.
    dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("cct_pose_commander"),
            "launch", "dashboard.launch.py"])),
        launch_arguments={
            "port": LaunchConfiguration("dashboard_port"),
            "commander_ns": PythonExpression(
                ["'/cct_pose_commander_' + '",
                 LaunchConfiguration("instance_name"), "'"]),
            "base_frame": LaunchConfiguration("dashboard_base_frame"),
        }.items(),
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration("dashboard_port"), "' != ''"])),
    )

    return LaunchDescription(args + [node, dashboard])
