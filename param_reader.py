#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
import time


def get_param_from_other_node(self_node, target_node_name, param_name):
    """
    Get a parameter value from another node.
    
    Args:
        self_node: The node making the request
        target_node_name: Name of the node that owns the parameter
        param_name: Name of the parameter to retrieve
        
    Returns:
        Parameter value or None if not available
    """
    client = self_node.create_client(GetParameters, f'/{target_node_name}/get_parameters')
    if not client.wait_for_service(timeout_sec=1.0):
        self_node.get_logger().error(f'Service {target_node_name}/get_parameters not available')
        return None
    
    request = GetParameters.Request()
    request.names = [param_name]
    future = client.call_async(request)
    
    # Wait for response
    rclpy.spin_until_future_complete(self_node, future)
    
    if future.done():
        try:
            response = future.result()
            if response.values:
                return response.values[0]
            else:
                self_node.get_logger().warning(f'Parameter {param_name} not found')
                return None
        except Exception as e:
            self_node.get_logger().error(f'Service call failed: {str(e)}')
            return None
    else:
        self_node.get_logger().error('Service call timed out')
        return None


class ParameterConsumerNode(Node):
    """Node that retrieves parameters from another node."""

    def __init__(self):
        super().__init__('parameter_consumer')
        
        # Source node name that has the parameters we want to access
        self.source_node_name = 'parameter_publisher'
        
        # Create a timer to periodically fetch and display parameters
        self.timer = self.create_timer(2.0, self.fetch_parameters)
        
        # Wait a moment for the parameter publisher to start up
        self.get_logger().info('Parameter consumer node initialized')
        self.get_logger().info(f'Will fetch parameters from {self.source_node_name}')

    def fetch_parameters(self):
        """Fetch and display parameters from the publisher node."""
        # List of parameters to fetch
        param_names = ['string_param', 'int_param', 'double_param', 'bool_param']
        
        self.get_logger().info('Fetching parameters from other node...')
        
        for param_name in param_names:
            value = get_param_from_other_node(self, self.source_node_name, param_name)
            if value is not None:
                self.get_logger().info(f'Retrieved {param_name}: {value.string_value or value.integer_value or value.double_value or value.bool_value}')
            else:
                self.get_logger().warning(f'Failed to retrieve {param_name}')


def main(args=None):
    rclpy.init(args=args)
    node = ParameterConsumerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()