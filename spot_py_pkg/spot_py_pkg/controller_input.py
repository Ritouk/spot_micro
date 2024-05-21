#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import getch
from example_interfaces.msg import String

class ControllerInputNode(Node):

    def __init__(self):
        super().__init__("controller_input")
        self.get_logger().info("Controller Input has been Started")
        
        self.control_input_publisher_ = self.create_publisher(String, "control_input", 10)
        while(True):
            user_input = ''
            self.get_logger().info(f'Waiting for input')
            user_input = getch.getch()
            self.get_logger().info(f'Your single Character input was: {user_input}')
            msg = String()
            msg.data = user_input
            self.control_input_publisher_.publish(msg)
        


def main(args=None):
    rclpy.init(args=args)
    node = ControllerInputNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
