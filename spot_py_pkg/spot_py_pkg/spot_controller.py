import rclpy
from rclpy.node import Node

from spot_py import servo_fun_lib

from example_interfaces.msg import String

class SmartphoneNode(Node):
    def __init__(self):
        super().__init__("spot_controller")
        self.subscriber_ = self.create_subscription(
            String, "control_input", self.callback_robot_news, 10)
        self.get_logger().info("Robot Control has been Started")

    def callback_robot_news(self, msg):
        self.get_logger().info(f"Control has been recieved: {msg.data}")
        self.switch(msg.data)
    
    def switch(self,char):
        if char == "q":
            servo_fun_lib.lie_down()
        elif char == "u":
            servo_fun_lib.get_up()
        elif char == "l":
            servo_fun_lib.low()
        elif char == "c":
            servo_fun_lib.calib_angles(90)
        else:
            self.get_logger().info(f"No command exist for this input: {char}")
        self.get_logger().info(f"Ready for next Input")

def main(args=None):
    rclpy.init(args=args)
    node = SmartphoneNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
