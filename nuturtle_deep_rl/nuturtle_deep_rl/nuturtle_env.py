import enum
import time
from typing import Optional
import scipy.stats
import numpy as np


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


class NuTurtleEnv(gymnasium.Env):

    def __init__(self, node: Node) -> None:
        super().__init__()
        self.node = node
        self.action_space = spaces.Discrete(n=3, seed=42)
        self.observation_space = spaces.Box(
            low=np.array([-1.0, -0.5, -np.pi]),
            high=np.array([4.0, 4.5, np.pi]),
            dtype=np.float64,
            shape=(3,),
        )
        self.reward_range = (-1.0, 1.0)
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

    def sub_collide_callback(self, msg: Bool):
        if msg.data:
            self.collide = True

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
            theta = self.normalize_angle(e[2])

            self.state = np.array([x, y, theta])

    def normalize_angle(self, angle):
        output = np.arctan2(np.sin(angle), np.cos(angle))
        return output

    def compute_reward(self, done):
        x, y, _ = self.state
        d = np.sqrt(np.square(x) + np.square(y - 4.0))

        if done:
            return -1.0

        elif d < 0.5:
            return 1.0

        else:
            return 0.0

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ):
        super().reset(seed=seed)
        request = Empty.Request()

        future = self.cli_reset_turtle.call_async(request=request)
        rclpy.spin_until_future_complete(node=self.node, future=future)

        if not future.done():
            self.node.get_logger().error("ERROR: The service call is not complete")
            raise RuntimeError("Service not complete")

        self.state = np.zeros(3, dtype=np.float64)

        return self.state, {}

    def is_done(self):
        if self.collide:
            self.node.get_logger().info("Collision")
            self.collide = False
            return True

        else:
            return False
