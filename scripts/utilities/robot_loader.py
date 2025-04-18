#!/usr/bin/python3
print("Loaded robot loader")

import json
import os
import sys
import rclpy
from rclpy.node import Node
import ament_index_python.packages
import importlib
from difflib import SequenceMatcher

class RobotLoader(Node):
    def __init__(self):
        super().__init__('robot_loader')
        
        self.robots = []
        self.robot_list = []
        
        # Get package paths using ROS2 approach
        try:
            agents_path = ament_index_python.packages.get_package_share_directory('robogpt_agents')
            vision_path = ament_index_python.packages.get_package_share_directory('robogpt_vision')
            driver_path = ament_index_python.packages.get_package_share_directory('robot_drivers')
            
            self.config_path = os.path.join(agents_path, "config/robot_config/bot_details.json")
            self.vision_path = os.path.join(vision_path, "config/vision_config.json")
            self.driver_path = driver_path
            self.robotgpt_config = os.path.join(agents_path, "config/robot_config/robogpt.json")
        except ament_index_python.packages.PackageNotFoundError as e:
            self.get_logger().error(f"Package not found: {e}")

    def is_robot_connected(self, ip: str) -> bool:
        # Ping the IP address with 1 packet and wait for a response
        command = f"ping -c 1 -W 1 {ip} > /dev/null 2>&1"
        response = os.system(command)
        return response == 0
    
    def get_matched_robot(self, robot_name: str) -> str:
        with open(self.config_path, 'r') as file:
            bot_dict = json.load(file)
        bot_list = bot_dict["bot_list"]
        best_match = max(bot_list, key=lambda obj: SequenceMatcher(None, robot_name.lower(), obj).ratio(), default=None)
        return best_match

    def load_robots(self):
        try:
            # Use ROS2 parameter API
            robot_model = self.declare_parameter('robot_model', 'ec63').value
            sim_flag = self.declare_parameter('use_sim', False).value

            with open(self.config_path, "r") as file:
                data = json.load(file)

            bot = self.get_matched_robot(robot_name=robot_model)
            self.robot_list.append(robot_model)
            self.get_logger().info(f"Updating the wrapper of {bot}")
            wrapper_path = "robot_drivers.wrappers.sim_wrapper" if sim_flag else f"robot_drivers.wrappers.{bot}_wrapper"

            # Append the parent directory of 'wrappers' to sys.path
            wrappers_parent_dir = os.path.dirname(self.driver_path)
            if wrappers_parent_dir not in sys.path:
                sys.path.append(wrappers_parent_dir)

            bot_control = importlib.import_module(wrapper_path)
            wrapper = getattr(bot_control, 'bot_wrapper')

            if sim_flag:
                robot_config = data[robot_model]
                self.robots.append(wrapper(
                    robot_config["robot_group"],
                    robot_config["gripper_group"],
                    robot_config["time_out"],
                    robot_config["gripper_enable"]
                ))
            else:
                if self.is_robot_connected(data[robot_model]["robot_ip"]):
                    self.robots.append(wrapper(data[robot_model]["robot_ip"]))
                else:
                    self.get_logger().warn("Please check the robot connection and try again")

        except Exception as err:
            self.get_logger().error(f"Could not load robot due to {err}")
        
        return self.robot_list, self.robots