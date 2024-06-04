import numpy as np

from gymnasium import spaces

import rclpy
from rclpy.node import Node


from geometry_msgs.msg import Twist

from nuturtle_deep_rl.nuturtle_env import NuTurtleEnv

FORWARD = 0
LEFT = 1
RIGHT = 2


class NuTurtleDQNEnv(NuTurtleEnv):

    def __init__(self, node: Node) -> None:
        super().__init__(node=node)
        self.action_space = spaces.Discrete(n=3, seed=42)

    def parse_action(self, action):
        if action == FORWARD:
            return np.array([0.22, 0], dtype=np.float64)

        elif action == LEFT:
            return np.array([0.0, 2.0], dtype=np.float64)

        elif action == RIGHT:
            return np.array([0.0, -2.0], dtype=np.float64)

        else:
            raise RuntimeError("Invalid action")

    def step(self, action):
        u = self.parse_action(action)

        cmd = Twist()
        cmd.linear.x = u[0]
        cmd.angular.z = u[1]

        self.pub_cmd_vel.publish(cmd)

        rclpy.spin_once(node=self.node)
        self.update_state()

        done = self.is_done()
        reward = self.compute_reward(done)

        return self.state, reward, done, False, {}
