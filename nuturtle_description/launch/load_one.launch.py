from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    Shutdown,
    GroupAction,
    SetLaunchConfiguration,
)
from launch_ros.actions import PushROSNamespace
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    TextSubstitution,
)

package_name = "nuturtle_description"


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="use_rviz",
                default_value="true",
                choices=["true", "false"],
                description="Where to launch the rviz to visualize",
            ),
            DeclareLaunchArgument(
                name="use_jsp",
                default_value="true",
                choices=["true", "false"],
                description="Whether to use the joint state publisher",
            ),
            DeclareLaunchArgument(
                name="color",
                default_value="purple",
                choices=["red", "green", "blue", "purple"],
                description="The color of the base link",
            ),
            SetLaunchConfiguration(
                name="config_filename",
                value=[
                    TextSubstitution(text="basic_"),
                    LaunchConfiguration("color"),
                    TextSubstitution(text=".rviz"),
                ],
            ),
            GroupAction(
                actions=[
                    PushROSNamespace(LaunchConfiguration("color")),
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2",
                        condition=IfCondition(LaunchConfiguration("use_rviz")),
                        arguments=[
                            "-d",
                            PathJoinSubstitution(
                                [
                                    FindPackageShare(package=package_name),
                                    "config",
                                    LaunchConfiguration("config_filename"),
                                ]
                            ),
                            "-f",
                            PathJoinSubstitution(
                                [
                                    LaunchConfiguration("color"),
                                    TextSubstitution(text="base_link"),
                                ]
                            ),
                        ],
                        on_exit=Shutdown(),
                    ),
                    Node(
                        package="joint_state_publisher",
                        executable="joint_state_publisher",
                        name="joint_state_publisher",
                        parameters=[{"rate": 100}],
                        condition=IfCondition(LaunchConfiguration("use_jsp")),
                        on_exit=Shutdown(),
                    ),
                    Node(
                        package="robot_state_publisher",
                        executable="robot_state_publisher",
                        name="robot_state_publisher",
                        parameters=[
                            {
                                "robot_description": Command(
                                    [
                                        ExecutableInPackage("xacro", "xacro"),
                                        TextSubstitution(text=" "),
                                        PathJoinSubstitution(
                                            [
                                                FindPackageShare(package=package_name),
                                                "urdf",
                                                "turtlebot3_burger.urdf.xacro",
                                            ]
                                        ),
                                        TextSubstitution(text=" color:="),
                                        LaunchConfiguration("color"),
                                    ]
                                ),
                                "frame_prefix": PathJoinSubstitution(
                                    [LaunchConfiguration("color"), ""]
                                ),
                            }
                        ],
                        on_exit=Shutdown(),
                    ),
                ]
            ),
        ]
    )
