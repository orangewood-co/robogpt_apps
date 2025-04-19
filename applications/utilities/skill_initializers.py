#!/usr/bin/python3
print("Loaded skill initializers")

import random
import numpy as np
import time
import sys, os
import json
import rclpy
import tf2_ros
import ament_index_python.packages
import pusher  
import importlib
import geometry_msgs.msg
import tf2_geometry_msgs
from difflib import SequenceMatcher
# from robogpt_applications.scripts.utilities import utils
from langchain.tools import BaseTool
from typing import Type, List, Dict, Optional
from pydantic import BaseModel, Field, ConfigDict
# from robogpt_applications.scripts.utilities.helper_services import ExternalServices
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped
import tf_transformations

# # Define file paths for configuration and results
# try:
#     # Use ament_index to find packages in ROS2
#     base_dir = ament_index_python.packages.get_package_share_directory('robogpt_agents')
#     driver_path = ament_index_python.packages.get_package_share_directory('robot_drivers')
#     vision_path = ament_index_python.packages.get_package_share_directory('robogpt_vision')
#     app_path = ament_index_python.packages.get_package_share_directory('robogpt_apps')
# except ament_index_python.packages.PackageNotFoundError as e:
#     print(f"Package not found: {e}")
#     # Fallback to environment-based paths if needed
#     base_dir = os.environ.get('ROBOGPT_AGENTS_DIR', '/path/to/robogpt_agents')
#     driver_path = os.environ.get('ROBOT_DRIVERS_DIR', '/path/to/robot_drivers')
#     vision_path = os.environ.get('ROBOGPT_VISION_DIR', '/path/to/robogpt_vision')
#     app_path = os.environ.get('ROBOGPT_APPS_DIR', '/path/to/robogpt_apps')

# # Configuration file paths
# robot_home_file_path = os.path.join(base_dir, "config/robot_config/robot_pose.json")
# robot_joint_file_path = os.path.join(base_dir, "config/robot_config/robot_joints.json")
# robotgpt_config = os.path.join(base_dir, "config/robot_config/robogpt.json")
# tour_paths = os.path.join(base_dir, "config/tour_scripts")
# object_details = os.path.join(vision_path, "config/vision_config.json")
# env_path = os.path.join(base_dir, "config/.demo_env")
# keys = utils.load_env_variables(env_path)
class Colors:
    # Reset
    RESET = "\033[0m"
    
    # Regular colors
    BLACK = "\033[30m"
    RED = "\033[31m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"
    MAGENTA = "\033[35m"
    CYAN = "\033[36m"
    WHITE = "\033[37m"
    