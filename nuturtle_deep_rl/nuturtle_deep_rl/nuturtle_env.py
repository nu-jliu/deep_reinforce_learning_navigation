import numpy as np

import gym
from gym import spaces
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
import time
import scipy.stats


class NuTurtleEnv(gym.Env):

    def __init__(self, node: Node) -> None:
        super().__init__()
        self.node = node
        self.action_space = spaces.Box(
            low=np.array([0.0, -1.0]),
            high=np.array([0.2, 1.0]),
            dtype=np.float64,
        )
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            dtype=np.float64,
            shape=(3,),
        )
        self.reward_range = (-np.inf, np.inf)
        # self.time_
        self.odom: Odometry = None
        self.state = np.zeros(3)
        self.collide = False

        self.node.declare_parameter(
            "turn.x",
            3.5,
            ParameterDescriptor(description="X turning coordinate"),
        )
        self.node.declare_parameter(
            "turn.y",
            3.5,
            ParameterDescriptor(description="Y turning coordinate"),
        )
        self.node.declare_parameter(
            "gain.d",
            10.0,
            ParameterDescriptor(description="Gain on distance to desired trajectory"),
        )
        self.node.declare_parameter(
            "gain.target",
            2.5,
            ParameterDescriptor(description="Gain on target position"),
        )
        self.node.declare_parameter(
            "gain.linear",
            5.0,
            ParameterDescriptor(description="Gain on linear speed"),
        )
        self.node.declare_parameter(
            "gain.angular",
            1.0,
            ParameterDescriptor(description="Gain on linear speed"),
        )

        self.turn_x = (
            self.node.get_parameter("turn.x").get_parameter_value().double_value
        )
        self.turn_y = (
            self.node.get_parameter("turn.y").get_parameter_value().double_value
        )
        self.gain_d = (
            self.node.get_parameter("gain.d").get_parameter_value().double_value
        )
        self.gain_target = (
            self.node.get_parameter("gain.target").get_parameter_value().double_value
        )
        self.gain_linear = (
            self.node.get_parameter("gain.linear").get_parameter_value().double_value
        )
        self.gain_angular = (
            self.node.get_parameter("gain.angular").get_parameter_value().double_value
        )

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
        # self.node.get_logger().warn(f"got odom: {msg.pose.pose.position}")
        # self.node.get_logger().info(f"State: {self.state}")

    def sub_collide_callback(self, msg: Bool):
        if msg.data:
            self.collide = True
        # self.node.get_logger().info(f"Collsion: {self.collide}")

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
            # self.node.get_logger().info(f"euler angle: {e}")
            theta = self.normalize_angle(e[2])

            self.state = np.array([x, y, theta])

    def normalize_angle(self, angle):
        output = np.arctan2(np.sin(angle), np.cos(angle))
        return output

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
        min_ind = np.argmin(np.array([d1, d2, d3]))

        d_angle = 0
        if min_ind == 0:
            d_angle = 0

        elif min_ind == 1:
            d_angle = np.pi / 2.0

        elif min_ind == 2:
            d_angle = np.pi

        e_angle = self.normalize_angle(theta - d_angle)

        # reward -= self.gain_d * np.sqrt(d)
        # reward -= self.gain_target * np.sum(
        #     np.square(self.state[:2] - np.array([0, self.turn_y]))
        # )
        # reward += self.gain_linear * action[0] + self.gain_angular * np.square(
        #     action[1]
        # )
        if np.sqrt(d) > 0.2:
            reward -= 10
        else:
            reward += scipy.stats.norm.pdf(np.sqrt(d), 0, 0.2)

        if np.fabs(e_angle) > 0.5:
            reward -= 10
        else:
            reward += scipy.stats.norm.pdf(np.fabs(e_angle), 0, 0.2)

        # self.node.get_logger().info(f"Reward: {reward}")
        # self.node.get_logger().info(f"d1: {d1}")
        # self.node.get_logger().info(f"d2: {d2}")
        # self.node.get_logger().info(f"d3: {d3}")
        # self.node.get_logger().info(f"d: {np.sqrt(d)}")
        # self.node.get_logger().info(f"angle: {e_angle}")
        return reward

    def reset(self):
        request = Empty.Request()

        future = self.cli_reset_turtle.call_async(request=request)
        rclpy.spin_until_future_complete(node=self.node, future=future)

        self.state = np.zeros(3, dtype=np.float64)

        return self.state

    def is_done(self):
        if self.collide:
            self.node.get_logger().info("Collision")
            self.collide = False
            return True

        else:
            return False
