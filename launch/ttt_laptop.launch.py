"""Laptop side: MoveIt + the tic-tac-toe server in one command.

    source /opt/ros/jazzy/setup.bash
    source ~/<your roarm workspace>/install/setup.bash
    cd ~/<ttt folder>
    ros2 launch launch/ttt_laptop.launch.py

Everything here runs in the GAME's ROS domain (default 43), the same as the
game arm's driver on the Pi, so it can never move the bucket arm (domain 42).

The Pi only runs the drivers (launch/ttt_pi.launch.py). Arguments:
    domain:=43                                                game arm's ROS_DOMAIN_ID
    moveit_pkg:=roarm_moveit  moveit_launch:=demo.launch.py   your MoveIt bringup
    rviz:=true|false                                          passed to it as use_rviz
    ttt_dir:=<path>                                           this folder (default: here)
    config:=<file>                                            config file (default: ttt_dir/config.yaml)
"""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _setup(context):
    from ament_index_python.packages import get_package_share_directory
    pkg = LaunchConfiguration("moveit_pkg").perform(context)
    file = LaunchConfiguration("moveit_launch").perform(context)
    ttt_dir = LaunchConfiguration("ttt_dir").perform(context)
    config = LaunchConfiguration("config").perform(context) or os.path.join(ttt_dir, "config.yaml")
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg), "launch", file)),
        launch_arguments={"use_rviz": LaunchConfiguration("rviz").perform(context)}.items(),
    )
    server = ExecuteProcess(
        cmd=["python3", "-m", "ttt.server", "--backend", "ros", "--config", config],
        cwd=ttt_dir, output="screen", emulate_tty=True,
    )
    # give move_group a few seconds to come up before the game connects to it
    return [moveit, TimerAction(period=5.0, actions=[server])]


def generate_launch_description():
    here = str(Path(__file__).resolve().parents[1])
    return LaunchDescription([
        DeclareLaunchArgument("moveit_pkg", default_value="roarm_moveit"),
        DeclareLaunchArgument("moveit_launch", default_value="demo.launch.py"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("ttt_dir", default_value=here),
        DeclareLaunchArgument("config", default_value=""),
        DeclareLaunchArgument("domain", default_value="43"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("domain")),
        OpaqueFunction(function=_setup),
    ])
