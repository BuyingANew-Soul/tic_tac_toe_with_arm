"""Pi side: the three device drivers, each robot in its own ROS domain.

    source /opt/ros/jazzy/setup.bash
    source ~/<roarm workspace>/install/setup.bash      # roarm_driver
    source ~/<AUWO-testbed>/install/setup.bash         # oak_d_bringup (only if camera:=true)
    ros2 launch ~/ttt/launch/ttt_pi.launch.py

    game arm   (table, plays tic-tac-toe)    ROS_DOMAIN_ID 43   /dev/roarm_game
    bucket arm (UGV, reward excavation)      ROS_DOMAIN_ID 42   /dev/roarm_ugv
    OAK-D Pro  (UGV camera)                  ROS_DOMAIN_ID 42

Why two domains: both arms use the same topic names (/roarm/joint_states,
/move_action ...). In one domain each arm would also follow the other's
commands. Separate domains keep them apart with no renaming.

Arguments (all optional):
    game:=true|false  ugv:=true|false  camera:=true|false
    game_port:=/dev/roarm_game  ugv_port:=/dev/roarm_ugv
    game_domain:=43  ugv_domain:=42
    joint_states:=/roarm/joint_states     topic the drivers follow (what MoveIt publishes)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def arm_driver(name, port, topic):
    # Waveshare's driver subscribes to the relative name "joint_states"; ported
    # versions often use /roarm/joint_states. Remap both onto the chosen topic.
    return Node(
        package="roarm_driver", executable="roarm_driver", name=name,
        parameters=[{"serial_port": port, "baud_rate": 115200}],
        remappings=[("joint_states", topic), ("/roarm/joint_states", topic)],
        output="screen", emulate_tty=True,
    )


def generate_launch_description():
    lc = LaunchConfiguration
    args = [
        DeclareLaunchArgument("game", default_value="true"),
        DeclareLaunchArgument("ugv", default_value="true"),
        DeclareLaunchArgument("camera", default_value="true"),
        DeclareLaunchArgument("game_port", default_value="/dev/roarm_game"),
        DeclareLaunchArgument("ugv_port", default_value="/dev/roarm_ugv"),
        DeclareLaunchArgument("game_domain", default_value="43"),
        DeclareLaunchArgument("ugv_domain", default_value="42"),
        DeclareLaunchArgument("joint_states", default_value="/roarm/joint_states"),
    ]

    game = GroupAction(
        condition=IfCondition(lc("game")), scoped=True,
        actions=[
            SetEnvironmentVariable("ROS_DOMAIN_ID", lc("game_domain")),
            arm_driver("roarm_driver_game", lc("game_port"), lc("joint_states")),
        ],
    )
    ugv = GroupAction(
        condition=IfCondition(lc("ugv")), scoped=True,
        actions=[
            SetEnvironmentVariable("ROS_DOMAIN_ID", lc("ugv_domain")),
            arm_driver("roarm_driver_ugv", lc("ugv_port"), lc("joint_states")),
        ],
    )
    camera = GroupAction(
        condition=IfCondition(lc("camera")), scoped=True,
        actions=[
            SetEnvironmentVariable("ROS_DOMAIN_ID", lc("ugv_domain")),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("oak_d_bringup"), "launch", "oak_d_pro.launch.py"]))),
        ],
    )
    return LaunchDescription(args + [game, ugv, camera])
