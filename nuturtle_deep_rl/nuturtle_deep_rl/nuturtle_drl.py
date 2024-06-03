import os
import numpy as np

import gym
from gym import spaces
from stable_baselines3 import PPO, DQN

import rclpy
from rclpy.node import Node

from tf2_ros import TransformListener, Buffer

from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid

from nuturtle_deep_rl.nuturtle_ppo_env import NuTurtlePPOEnv
from nuturtle_deep_rl.nuturtle_dqn_env import NuTurtleDQNEnv


class NuTurtleDRL(Node):
    def __init__(self):
        super().__init__("nuturtle_deeprl")

        # ---------- ROS Setup ----------
        self.declare_parameter(
            "pkg_share_dir",
            "",
            ParameterDescriptor(description="Package share directory"),
        )
        self.declare_parameter(
            "log_dirname",
            "turtlebot3.log",
            ParameterDescriptor(description="Log directory"),
        )
        self.declare_parameter(
            "algorithm",
            "ppo",
            ParameterDescriptor(
                description="Algorithm used for deep reinforcement learning"
            ),
        )

        self.pkg_share_dir = (
            self.get_parameter("pkg_share_dir").get_parameter_value().string_value
        )
        self.log_dirname = (
            self.get_parameter("log_dirname").get_parameter_value().string_value
        )
        self.algorithm = (
            self.get_parameter("algorithm").get_parameter_value().string_value
        )

        # self.timer = self.create_timer(0.01, self.timer_callback)
        self.log_path = os.path.join(
            self.pkg_share_dir,
            f"{self.algorithm}_{self.log_dirname}",
        )
        self.get_logger().info("Setting up environment ...")

        if self.algorithm == "dqn":
            self.env_dqn = NuTurtleDQNEnv(node=self)
            self.model = DQN(
                policy="MlpPolicy",
                env=self.env_dqn,
                verbose=1,
                # learning_rate=0.01,
                tensorboard_log=self.log_path,
            )

            net = self.model.q_net
            for name, layer in net.named_children():
                self.get_logger().info(f"{name}: {layer}")

        elif self.algorithm == "ppo":
            self.env_ppo = NuTurtlePPOEnv(node=self)
            self.model = PPO(
                policy="MlpPolicy",
                env=self.env_ppo,
                verbose=1,
                # learning_rate=0.05,
                # n_steps=10,
                # n_epochs=10,
                # batch_size=10,
                gamma=0.95,
                tensorboard_log=self.log_path,
            )

            net = self.model.policy.action_net

            for name, layer in net.named_children():
                self.get_logger().info(f"{name}, {layer}")

        # self.get_logger().info(f"{self.model.policy}")
        self.get_logger().warn("Start training ...")
        # self.model.set_logger(self.get_logger())
        self.model.learn(total_timesteps=int(1e15), progress_bar=True, log_interval=1)
        self.model.save(
            os.path.join(
                self.pkg_share_dir,
                f"{self.algorithm}_turtlebot3",
            )
        )
        self.get_logger().info("Training finished")
        # self.model = PPO(env=self.env)

    # def timer_callback(self):
    #     pass


def main(args=None):
    rclpy.init(args=args)
    node_drl = NuTurtleDRL()
    rclpy.spin(node=node_drl)

    node_drl.destroy_node()
    rclpy.shutdown()
