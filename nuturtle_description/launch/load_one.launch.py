from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage

package_name = "nuturtle_description"


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="use_rviz",
                default_value="true",
                description="Where to launch the rviz to visualize",
            ),
            DeclareLaunchArgument(
                name="use_jsp",
                default_value="true",
                description="Whether to use the joint state publisher",
            ),
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
                            "basic_turple.rviz",
                        ]
                    ),
                ],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                condition=IfCondition(LaunchConfiguration("use_jsp")),
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
                                " ",
                                PathJoinSubstitution(
                                    [
                                        FindPackageShare(package=package_name),
                                        "urdf",
                                        "turtlebot3_burger.urdf.xacro",
                                    ]
                                ),
                            ]
                        )
                    }
                ],
                on_exit=Shutdown(),
            ),
        ]
    )
