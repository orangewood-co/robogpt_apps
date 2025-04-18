#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class ParameterPublisherNode(Node):
    """Node that declares and publishes parameters for other nodes to access."""

    def __init__(self):
        super().__init__('parameter_publisher')
        
        # Declare parameters with default values
        self.declare_parameter('string_param', 'Hello ROS2')
        self.declare_parameter('int_param', 42)
        self.declare_parameter('double_param', 3.14159)
        self.declare_parameter('bool_param', True)
        
        # Print out the parameters we've set
        self.get_logger().info('Parameters initialized')
        self.get_logger().info(f'string_param: {self.get_parameter("string_param").value}')
        self.get_logger().info(f'int_param: {self.get_parameter("int_param").value}')
        self.get_logger().info(f'double_param: {self.get_parameter("double_param").value}')
        self.get_logger().info(f'bool_param: {self.get_parameter("bool_param").value}')
        
        # Create a timer to periodically display parameter values (optional)
        self.timer = self.create_timer(5.0, self.timer_callback)

    def timer_callback(self):
        """Display current parameter values periodically."""
        self.get_logger().info('Current parameter values:')
        self.get_logger().info(f'string_param: {self.get_parameter("string_param").value}')
        self.get_logger().info(f'int_param: {self.get_parameter("int_param").value}')
        self.get_logger().info(f'double_param: {self.get_parameter("double_param").value}')
        self.get_logger().info(f'bool_param: {self.get_parameter("bool_param").value}')


def main(args=None):
    rclpy.init(args=args)
    node = ParameterPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()