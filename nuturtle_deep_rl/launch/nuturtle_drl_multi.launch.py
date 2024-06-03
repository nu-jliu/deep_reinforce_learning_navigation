import os

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import PushROSNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package = "nuturtle_deep_rl"
    num_env = int(os.getenv("NUM_GYM_ENV"))
    launch_descriptions = []

    for i in range(num_env):
        # namespace = f"nuturtle_ns_{i+1}"
        launch_descriptions.append(
            GroupAction(
                actions=[
                    # PushROSNamespace(namespace=namespace),
                    SetEnvironmentVariable(name="ROS_DOMAIN_ID", value=f"{i + 1}"),
                    IncludeLaunchDescription(
                        launch_description_source=PathJoinSubstitution(
                            substitutions=[
                                FindPackageShare(package=package),
                                "launch",
                                "nuturtle_drl.launch.xml",
                            ]
                        ),
                        launch_arguments={"drl_algorithm": "ppo"}.items(),
                    ),
                ]
            )
        )

    for i in range(num_env):
        # namespace = f"nuturtle_ns_{i+1}"
        launch_descriptions.append(
            GroupAction(
                actions=[
                    # PushROSNamespace(namespace=namespace),
                    SetEnvironmentVariable(
                        name="ROS_DOMAIN_ID",
                        value=f"{num_env + i + 1}",
                    ),
                    IncludeLaunchDescription(
                        launch_description_source=PathJoinSubstitution(
                            substitutions=[
                                FindPackageShare(package=package),
                                "launch",
                                "nuturtle_drl.launch.xml",
                            ]
                        ),
                        launch_arguments={"drl_algorithm": "dqn"}.items(),
                    ),
                ]
            )
        )

    # launch_descriptions.append(
    #     IncludeLaunchDescription(
    #         launch_description_source=PathJoinSubstitution(
    #             substitutions=[
    #                 FindPackageShare(package=package),
    #                 "launch",
    #                 "ppo_turtle_tensorboard.launch.xml",
    #             ]
    #         )
    #     )
    # )

    return LaunchDescription(launch_descriptions)
