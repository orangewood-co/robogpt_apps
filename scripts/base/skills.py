import rclpy.logging
from robogpt_applications.scripts.utilities.skill_initializers import *
from typing import ClassVar, Optional, Dict, Any
import time
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
    
# Data model for the 'delay' tool
class delay_definition(BaseModel):
    delay: float = Field(description="Delay in seconds.")
    
    # Add this for Pydantic v2 compatibility
    model_config = ConfigDict(arbitrary_types_allowed=True)

# Implementation of the 'delay' tool
class delay_implementation(BaseTool):
    """Tool to add delay in the script."""
    # Add proper type annotations to all class attributes
    name: ClassVar[str] = "delay"
    description: ClassVar[str] = "Adds delay in the script."
    args_schema: ClassVar[Type[BaseModel]] = delay_definition
    return_direct: ClassVar[bool] = False  # Add this if BaseTool has it
    verbose: ClassVar[bool] = False  # Add this if BaseTool has it
    
    # If there are any other attributes from BaseTool, add them with ClassVar annotations

    def _run(self, delay: float, flag: bool = True) -> str:
        print("delay_implementation")
        time.sleep(delay)
        return f"added delay for {delay} seconds"
    
    def _arun(self, delay: float) -> str:
        time.sleep(delay)
        return f"added delay for {delay} seconds"
    
    
# Data model for the 'delay' tool
class move_robot_definition(BaseModel):

    # Add this for Pydantic v2 compatibility
    model_config = ConfigDict(arbitrary_types_allowed=True)

# Implementation of the 'delay' tool

# Implementation of the 'move_robot' tool
class move_robot_implementation(BaseTool):
    """Tool to add delay in the script."""
    # Add proper type annotations to all class attributes
    name: ClassVar[str] = "move_robot"
    description: ClassVar[str] = "move the robot to a particular pose"
    args_schema: ClassVar[Type[BaseModel]] = move_robot_definition
    return_direct: ClassVar[bool] = False  # Add this if BaseTool has it
    verbose: ClassVar[bool] = False  # Add this if BaseTool has it
    
    # If there are any other attributes from BaseTool, add them with ClassVar annotations

    def _run(self) -> str:
        print("Running the move command")
        
        # Initialize ROS if not already initialized
        if not rclpy.ok():
            rclpy.init()
        
        # Create a node for parameter access
        node = rclpy.create_node('move_robot_param_checker')
        
        try:
            # Create a parameter client to access parameters from robogpt_agents node
            target_node_name = '/robogpt_agent'  # Target node name is robogpt_agent
            param_client = rclpy.parameter.SynchronousParameterClient(node, target_node_name)
            
            # Wait for the service to be available
            print(f"{Colors.YELLOW}Waiting for parameter service from {target_node_name}...{Colors.RESET}")
            ready = param_client.wait_for_service(timeout_sec=5.0)
            if not ready:
                print(f"{Colors.RED}Parameter service not available for {target_node_name}{Colors.RESET}")
                return f"Error: Parameter service not available for {target_node_name}"
            
            print(f"{Colors.GREEN}Connected to {target_node_name} parameter service{Colors.RESET}")
            
            # Get parameters from robogpt_agents node
            try:
                # List of parameters to get from robogpt_agent
                param_names = ['use_case', 'client_id', 'reload_tools']
                params = param_client.get_parameters(param_names)
                
        
                # Extract values with default fallbacks
                param_values = {}
                for i, name in enumerate(param_names):
                    if i < len(params) and params[i].type != rclpy.parameter.ParameterType.NOT_SET:
                        param_values[name] = params[i].value
                    else:
                        param_values[name] = f"<not set>"
                
                # Print the parameters we got
                print(f"{Colors.GREEN}Parameters from {target_node_name}:{Colors.RESET}")
                for name, value in param_values.items():
                    print(f"{Colors.YELLOW}{name}: {Colors.CYAN}{value}{Colors.RESET}")
                
                # Example of using the parameters
                use_case = param_values.get('use_case', 'base')
                print(f"{Colors.MAGENTA}Current use case is: {use_case}{Colors.RESET}")
                
                return f"Successfully read parameters from {target_node_name}: {param_values}"
                
            except Exception as e:
                print(f"{Colors.RED}Error getting parameters: {e}{Colors.RESET}")
                return f"Error getting parameters: {str(e)}"
            
        except Exception as e:
            print(f"{Colors.RED}Error accessing parameter service: {e}{Colors.RESET}")
            return f"Error in move_robot: {str(e)}"
        finally:
            # Clean up the node but don't shut down rclpy
            node.destroy_node()
    
    def _arun(self, delay: float) -> str:
        time.sleep(delay)
        return f"added delay for {delay} seconds"