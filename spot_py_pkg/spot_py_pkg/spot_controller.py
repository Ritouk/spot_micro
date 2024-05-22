import rclpy
from rclpy.node import Node

from spot_py_pkg import spot_control_class

from example_interfaces.msg import String

class SpotControllerNode(Node):
    def __init__(self):
        super().__init__("spot_controller")
        self.subscriber_ = self.create_subscription(
            String, "control_input", self.callback_robot_news, 10)
        self.get_logger().info("Robot Control has been Started")
        
        self.spot = spot_control_class.SpotPosition()

    def callback_robot_news(self, msg):
        self.get_logger().info(f"Control has been recieved: {msg.data}")
        self.switch(msg.data)
    
    def switch(self,char):
        if char == "q":
            self.spot.lie_down()
        elif char == "u":
            self.spot.get_up()
        elif char == "l":
            self.spot.low()
        elif char == "c":
            pass
            #self.spot.calib_angles(90)
        else:
            self.get_logger().info(f"No command exist for this input: {char}")
        self.get_logger().info(f"Ready for next Input")

def main(args=None):
    rclpy.init(args=args)
    node = SpotControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
