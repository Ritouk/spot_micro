import rclpy
from rclpy.node import Node

from spot_py_pkg import spot_control_class

from example_interfaces.msg import String

class SpotControllerNode(Node):
    def __init__(self):
        super().__init__("spot_controller")
        self.subscriber_ = self.create_subscription(
            String, "control_input", self.callback_robot_news, 10)
        self.get_logger().info(
            "Robot Control has been Started, initializing position")
        
        self.spot = spot_control_class.SpotPosition()
        self.get_logger().info("Robot Ready for input")
    
    def __del__(self):
        print("Shutting down, putting robot in sleep")
        del self.spot
        print("Done, bye! ")
                
    def callback_robot_news(self, msg):
        self.get_logger().info(f"Control has been recieved: {msg.data}")
        self.switch(msg.data)
    
    def switch(self,char):
        if char == 'A':
            self.get_logger().info("FORWARD")
        elif char == 'D':
            self.get_logger().info("ROT LEFT")
        elif char == 'B':
            self.get_logger().info("BACKWARDS")
        elif char == 'C':
            self.get_logger().info("ROT RIGHT")
        elif char == "q":
            self.get_logger().info("lie Down")
            self.spot.lie_down(times=1)
        elif char == "u":
            self.get_logger().info("Get Up")
            self.spot.get_up(times=1)
        elif char == "l":
            self.get_logger().info("Low")
            self.spot.low(times=1)
        elif char == "s":
            self.get_logger().info("Sit")
            self.spot.sit(times = 0.5)
        elif char == "x":
            #NOT WORKING PROPERLY
            self.get_logger().info("Shutting down, putting robot in sleep")
            self.__del__()
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
