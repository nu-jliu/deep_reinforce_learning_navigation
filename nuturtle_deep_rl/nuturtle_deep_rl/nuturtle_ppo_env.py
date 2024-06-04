import numpy as np
from typing import Optional

import gymnasium
from gymnasium import spaces
from stable_baselines3 import PPO

import rclpy
from rclpy.node import Node

import rclpy.node
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion

from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from std_srvs.srv import Empty

from nuturtle_deep_rl.nuturtle_env import NuTurtleEnv


class NuTurtlePPOEnv(NuTurtleEnv):

    def __init__(self, node: Node) -> None:
        super().__init__(node=node)
        self.action_space = spaces.Box(
            low=np.array([0.0, -2.0]),
            high=np.array([0.22, 2.0]),
            dtype=np.float64,
        )

    def step(self, action):
        cmd = Twist()
        cmd.linear.x = action[0]
        cmd.angular.z = action[1]

        self.pub_cmd_vel.publish(cmd)

        rclpy.spin_once(node=self.node)
        self.update_state()

        done = self.is_done()
        reward = self.compute_reward(done)

        return self.state, reward, done, False, {}
