import numpy as np

import gym
from gym import spaces
from stable_baselines3 import PPO

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
            "turn_x",
            3.5,
            ParameterDescriptor(description="X turning coordinate"),
        )
        self.declare_parameter(
            "turn_y",
            3.5,
            ParameterDescriptor(description="Y turning coordinate"),
        )
        self.declare_parameter(
            "pkg_share_dir",
            "",
            ParameterDescriptor(description="Package share directory"),
        )

        self.turn_x = self.get_parameter("turn_x").get_parameter_value().double_value
        self.turn_y = self.get_parameter("turn_y").get_parameter_value().double_value
        self.pkg_share_dir = (
            self.get_parameter("pkg_share_dir").get_parameter_value().string_value
        )

        self.get_logger().info("Setting up environment ...")
        self.env = NuTurtleEnv(node=self, turn_x=self.turn_x, turn_y=self.turn_y)
        self.model = PPO(policy="MlpPolicy", env=self.env, verbose=1)
        self.get_logger().warn("Start training ...")
        self.model.learn(total_timesteps=100000)
        self.model.save(f"{self.pkg_share_dir}/ppo_turtlebot3")
        # self.model = PPO(env=self.env)


def main(args=None):
    rclpy.init(args=args)
    node_drl = NuTurtleDRL()
    rclpy.spin(node=node_drl)

    node_drl.destroy_node()
    rclpy.shutdown()
