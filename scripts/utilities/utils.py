#!/usr/bin/python3

import os
import sys
import cv2
import json
import time
import rclpy
from rclpy.node import Node
import dotenv
import pusher
import logging
import zipfile
import ament_index_python.packages
import datetime
from std_msgs.msg import Bool



def append_template_to_python(python_file_path, template_file_path):
    """
    Appends content from a template text file to a Python file.
    
    Parameters:
        python_file_path (str): Path to the target Python file
        template_file_path (str): Path to the template text file containing code to append
    """
    try:
        # Read template content
        with open(template_file_path, 'r') as template_file:
            template_content = template_file.read()
    except FileNotFoundError:
        print(f"Error: Template file '{template_file_path}' not found")
        return
    except PermissionError:
        print(f"Error: Permission denied to read template file '{template_file_path}'")
        return

    try:
        # Append to Python file
        with open(python_file_path, 'a') as python_file:
            python_file.write('\n' + template_content)
        print(f"Successfully appended template to '{python_file_path}'")
    except FileNotFoundError:
        print(f"Error: Directory for Python file '{python_file_path}' doesn't exist")
    except PermissionError:
        print(f"Error: Permission denied to write to Python file '{python_file_path}'")
    except Exception as e:
        print(f"Unexpected error: {str(e)}")

def load_env_variables(env_file):
    # Load the environment variables from the .env file into os.environ
    dotenv.load_dotenv(env_file)
    
    # Extract the environment variables from the .env file dynamically
    env_vars = {}
    with open(env_file, 'r') as file:
        for line in file:
            # Skip comments and blank lines
            line = line.strip()
            if line and not line.startswith('#'):
                key, value = line.split('=', 1)
                key = key.strip()
                value = value.strip()
                env_vars[key] = value
                os.environ[key] = value  # Set the environment variables
    
    # Return the environment variables as a dictionary
    return env_vars

class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')
        self.robot_logger = None
        self.opt_logger = None
        self.setup_loggers()
        
    def setup_loggers(self):
        '''
        Configures and initializes two loggers for logging robot state and operation information. 
        It creates separate log files for each logger and sets up the log format.

        Returns:
        robotlogger (logging.Logger): A logger for robot state information.
        logger (logging.Logger): A logger for operation information.
        '''

        # Creates a folder in /log based on current date
        folder = datetime.datetime.now().strftime('log_%d_%m_%Y')
        os.makedirs(os.getcwd()+"/logs/"+folder, exist_ok=True)

        self.robot_logger = logging.getLogger("robot state")
        # Configure the log file and format for this script
        file_handler = logging.FileHandler(os.getcwd()+"/logs/"+folder+"/robot_state.log")
        formatter = logging.Formatter('%(asctime)s,%(msecs)d %(name)s %(levelname)s %(message)s')
        file_handler.setFormatter(formatter)
        self.robot_logger.setLevel(logging.INFO)
        self.robot_logger.addHandler(file_handler)

        self.opt_logger = logging.getLogger("operation")
        # Configure the log file and format for this script
        file_handler = logging.FileHandler(os.getcwd()+"/logs/"+folder+"/operation.log")
        formatter = logging.Formatter('%(asctime)s,%(msecs)d %(name)s %(levelname)s %(message)s')
        self.opt_logger.setLevel(logging.INFO)
        file_handler.setFormatter(formatter)
        self.opt_logger.addHandler(file_handler)

        return self.robot_logger, self.opt_logger

# Initialize Logger Node when module is imported
logger_node = None
robot_logger = None
opt_logger = None

def init_loggers():
    global logger_node, robot_logger, opt_logger
    if not rclpy.ok():
        rclpy.init()
    if logger_node is None:
        logger_node = LoggerNode()
        robot_logger, opt_logger = logger_node.robot_logger, logger_node.opt_logger

# Initialize loggers (this will run when the module is imported)
init_loggers()



def get_current_objects():
    try:
        # Get path to detection file using ament_index in ROS2
        vision_path = ament_index_python.packages.get_package_share_directory('robogpt_vision')
        detection_file = os.path.join(vision_path, "config/owl/detection_results.json")
    except Exception:
        detection_file = "config/owl/detection_results.json"  
        
    # Load detection results
    with open(detection_file, "r") as file:
        detection_results = json.load(file)
    objects = []

    # Extract object names
    for key, detections in detection_results.items():
        for detection_key, detection in detections.items():
            objects.append(detection["detected_object"])

    # Create a formatted string
    if not objects:
        return "No objects detected in the frame."
    
    unique_objects = set(objects)  # Remove duplicates if needed
    object_list_str = ", ".join(unique_objects)
    formatted_string = f"I see the following object(s) in the frame: {object_list_str}."
    return object_list_str

class BoolPublisher(Node):
    def __init__(self):
        super().__init__('bool_publisher')
        self.publishers = {}  # Dictionary to store publishers by topic

    def publish_bool(self, val: bool, topic: str = "/pass_topic", queue_size: int = 1) -> None:
        """
        Publishes a boolean message on a specified ROS topic.
        """
        # Create a publisher if one doesn't exist for this topic
        if topic not in self.publishers:
            self.publishers[topic] = self.create_publisher(Bool, topic, queue_size)
            # Allow time for the publisher to register
            time.sleep(0.5)
            
        self.get_logger().info(f"Publishing bool: {val} on topic {topic}")
        msg = Bool()
        msg.data = val
        self.publishers[topic].publish(msg)

# Create a global instance of the bool publisher
bool_publisher = None

def init_bool_publisher():
    global bool_publisher
    if not rclpy.ok():
        rclpy.init()
    if bool_publisher is None:
        bool_publisher = BoolPublisher()

def publish_pass_bool(val: bool, topic: str = "/pass_topic", queue_size: int = 1) -> None:
    """
    Publishes a boolean message on a specified ROS topic.
    Wrapper function that uses the global BoolPublisher instance.
    """
    global bool_publisher
    if bool_publisher is None:
        init_bool_publisher()
    bool_publisher.publish_bool(val, topic, queue_size)

def get_skill_template():
    return '''
class {{ class_name }}_definition(BaseModel):
    object: str = Field(default = None, descrption = "the object or device to which robot needs to press button")
class {{ class_name }}_implementation(BaseTool):
    """Tool to tell the position of the object"""
    name = "{{ tool_name }}"
    description = "{{ tool_description }}"
    args_schema: Type[BaseModel] = {{ class_name }}_definition

    def _run(self, object:str = None):
        robot_ip = {{ robot_ip }}
        send_message_to_webapp_implementation()._run(message="Sure! I have instructed the robot to make popcorn")
'''

def extract_xml_from_zip(zip_path, extract_to_folder):
    """
    Extracts XML files from a given zip file into a subfolder named after the zip file.
    Saves their paths in a list.

    :param zip_path: Path to the zip file.
    :param extract_to_folder: Base folder where the files will be extracted.
    :return: A list of paths to the extracted XML files.
    """
    xml_paths = []  # To store paths of extracted XML files

    zip_file_name = os.path.basename(zip_path)  # Get the name of the zip file
    zip_file_name_without_ext = os.path.splitext(zip_file_name)[0]  # Remove the extension
    # Full path for the new subdirectory
    full_extract_path = os.path.join(extract_to_folder, zip_file_name_without_ext)

    # Create the subdirectory if it doesn't exist
    if not os.path.exists(full_extract_path):
        os.makedirs(full_extract_path)

    with zipfile.ZipFile(zip_path, 'r') as zip_ref:
        # Extract all files in the zip to the subdirectory
        zip_ref.extractall(full_extract_path)
        # Loop through the file names
        for file_name in zip_ref.namelist():
            # Check if the file is an XML
            if file_name.endswith('.xml'):
                # Save the full path of the extracted XML file
                xml_paths.append(os.path.join(full_extract_path, file_name))

    print("All the program files extracted")

    return xml_paths


def is_robot_connected(ip: str) -> bool:
    """
    Check if the robot is connected via Ethernet by pinging its IP address.

    Args:
        ip (str): IP address of the robot.

    Returns:
        bool: True if the robot is connected, False otherwise.
    """
    # Ping the IP address with 1 packet and wait for a response
    command = f"ping -c 1 -W 1 {ip} > /dev/null 2>&1"
    # Execute the command
    response = os.system(command)
    # Return True if the ping was successful (exit code 0), False otherwise
    return response == 0

# Initialize modules when loaded
init_loggers()
init_bool_publisher()


