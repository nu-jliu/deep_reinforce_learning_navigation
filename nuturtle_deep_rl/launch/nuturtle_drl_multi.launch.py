import os

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import PushROSNamespace, Node
from launch_ros.substitutions import FindPackageShare

PACKAGE_NAME = "nuturtle_deep_rl"


def generate_launch_description():
    num_env = int(os.getenv("NUM_GYM_ENV"))
    launch_descriptions = [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d",
                PathJoinSubstitution(
                    [
                        FindPackageShare(package=PACKAGE_NAME),
                        "config",
                        "drl_multi.rviz",
                    ]
                ),
            ],
        )
    ]

    for i in range(num_env):
        multi = "False" if i == 0 else "True"

        namespace = f"nuturtle_ns_{i + 1}"
        launch_descriptions.append(
            GroupAction(
                actions=[
                    PushROSNamespace(namespace=namespace),
                    IncludeLaunchDescription(
                        launch_description_source=PathJoinSubstitution(
                            substitutions=[
                                FindPackageShare(package=PACKAGE_NAME),
                                "launch",
                                "nuturtle_drl.launch.xml",
                            ]
                        ),
                        launch_arguments={
                            "drl_algorithm": "ppo",
                            "robot_name": f"bot_{i+1}",
                            "use_rviz": "False",
                            "multi": multi,
                        }.items(),
                    ),
                ]
            )
        )

    for i in range(num_env):
        namespace = f"nuturtle_ns_{num_env + i + 1}"
        launch_descriptions.append(
            GroupAction(
                actions=[
                    PushROSNamespace(namespace=namespace),
                    IncludeLaunchDescription(
                        launch_description_source=PathJoinSubstitution(
                            substitutions=[
                                FindPackageShare(package=PACKAGE_NAME),
                                "launch",
                                "nuturtle_drl.launch.xml",
                            ]
                        ),
                        launch_arguments={
                            "drl_algorithm": "dqn",
                            "robot_name": f"bot_{num_env+i+1}",
                            "use_rviz": "False",
                            "multi": "True",
                        }.items(),
                    ),
                ]
            )
        )

    return LaunchDescription(launch_descriptions)
