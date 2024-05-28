import numpy as np

import gym
from gym import spaces
from stable_baselines3 import PPO

import rclpy
from rclpy.node import Node

import rclpy.node
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from std_srvs.srv import Empty
import time


class NuTurtleEnv(gym.Env):

    def __init__(self, node: Node, turn_x: float, turn_y: float) -> None:
        super().__init__()
        self.node = node
        self.turn_x = turn_x
        self.turn_y = turn_y
        self.action_space = spaces.Box(
            low=np.array([-0.5, -0.5]),
            high=np.array([0.5, 0.5]),
            dtype=np.float64,
        )
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            dtype=np.float64,
            shape=(3,),
        )
        self.reward_range = (-np.inf, np.inf)
        self.odom: Odometry = None
        self.state = np.zeros(3)
        self.collide = False

        self.pub_cmd_vel = self.node.create_publisher(Twist, "cmd_vel", 10)

        self.sub_odom = self.node.create_subscription(
            Odometry,
            "nusim/odom",
            self.sub_odom_callback,
            10,
        )
        self.sub_collide = self.node.create_subscription(
            Bool,
            "nusim/collide",
            self.sub_collide_callback,
            10,
        )

        self.cli_reset_turtle = self.node.create_client(Empty, "nusim/reset")

        while not self.cli_reset_turtle.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Service not available, waiting again ...")

    def sub_odom_callback(self, msg: Odometry):
        self.odom = msg
        self.node.get_logger().warn(f"got odom: {msg.pose.pose.position}")
        # self.node.get_logger().info(f"State: {self.state}")

    def sub_collide_callback(self, msg: Bool):
        self.collide = msg.data
        self.node.get_logger().info(f"Collsion: {self.collide}")

    def update_state(self):
        if self.odom is None:
            self.state = np.zeros(3, dtype=np.float64)
            return

        else:
            x = self.odom.pose.pose.position.x
            y = self.odom.pose.pose.position.y

            qx = self.odom.pose.pose.orientation.x
            qy = self.odom.pose.pose.orientation.y
            qz = self.odom.pose.pose.orientation.z
            qw = self.odom.pose.pose.orientation.w

            q = (qx, qy, qz, qw)
            e = euler_from_quaternion(q)
            theta = e[2]

            self.state = np.array([x, y, theta])

    def step(self, action):
        cmd = Twist()
        cmd.linear.x = action[0]
        cmd.angular.z = action[1]

        self.pub_cmd_vel.publish(cmd)

        rclpy.spin_once(node=self.node)
        self.update_state()

        reward = self.compute_reward(action)
        done = self.is_done()

        return self.state, reward, done, {}

    def compute_reward(self, action):
        # self.node.get_logger().info(f"State: {self.state}")
        x, y, theta = self.state

        reward = 0

        d1 = np.square(y)
        d2 = np.square(x - self.turn_x)
        d3 = np.square(y - self.turn_y)

        if x < 0:
            d1 += np.square(x)
            d3 += np.square(x)

        if x > self.turn_x:
            d1 += np.square(x - self.turn_x)
            d3 += np.square(x - self.turn_x)

        if y < 0:
            d2 += np.square(y)

        if y > self.turn_y:
            d2 += np.square(y - self.turn_y)

        d = np.min(np.array([d1, d2, d3]))

        reward -= 5.0 * np.sqrt(d)
        reward -= 10.0 * np.sum(np.square(self.state[:2] - np.array([0, self.turn_y])))
        reward += 50 * np.sum(np.fabs(action))

        self.node.get_logger().info(f"Reward: {reward}, d: {d}")
        return reward

    def reset(self):
        request = Empty.Request()

        future = self.cli_reset_turtle.call_async(request=request)

        rclpy.spin_until_future_complete(node=self.node, future=future)

        self.state = np.zeros(3, dtype=np.float64)

        return self.state

    def is_done(self):
        if self.collide:
            return True

        else:
            return False
