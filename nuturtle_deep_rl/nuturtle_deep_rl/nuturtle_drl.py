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

from nuturtle_deep_rl.nuturtle_env import NuTurtleEnv


class NuTurtleDRL(Node):
    def __init__(self):
        super().__init__("nuturtle_deeprl")

        # ---------- ROS Setup ----------
        self.declare_parameter(
            "pkg_share_dir",
            "",
            ParameterDescriptor(description="Package share directory"),
        )

        self.pkg_share_dir = (
            self.get_parameter("pkg_share_dir").get_parameter_value().string_value
        )

        # self.timer = self.create_timer(0.01, self.timer_callback)
        self.log_path = os.path.join(self.pkg_share_dir, "ppo_turtlebot3.log")
        self.get_logger().info("Setting up environment ...")
        self.env = NuTurtleEnv(node=self)
        self.model = PPO(
            policy="MlpPolicy",
            env=self.env,
            verbose=1,
            learning_rate=0.05,
            # n_steps=10,
            n_epochs=10,
            # batch_size=10,
            # gamma=0.95,
            tensorboard_log=self.log_path,
        )
        # self.model = DQN(
        #     policy="MlpPolicy",
        #     env=self.env,
        #     verbose=1,
        #     learning_rate=0.01,
        # )
        self.get_logger().warn("Start training ...")
        # self.model.set_logger(self.get_logger())
        self.model.learn(total_timesteps=int(1e15), progress_bar=True)
        self.model.save(os.path.join(self.pkg_share_dir, "ppo_turtlebot3"))
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
