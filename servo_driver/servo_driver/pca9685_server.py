import rclpy
from rclpy.node import Node

from adafruit_servokit import ServoKit

from spot_interfaces.srv import SetServo


class PCA9685DriverServerNode(Node):

    def __init__(self):
        self.is_pca_working = False
        super().__init__("pca9685_server")
        self.srv = self.create_service(SetServo, 'set_servo', self.set_servo_callback)
        try:
            self.kit = ServoKit(channels=16)
            self.get_logger().info("pca9685 driver has been Started")
            self.is_pca_working = True
        except:
            self.get_logger().warn("Couldn't connect to PCA9685")
            return
        
        
    def set_servo_callback(self, request, response):
        if self.is_pca_working:
            try:
                servo_number = request.servo_number
                angle = request.angle
                self.kit.servo[servo_number].angle = angle
                response.success = True
                response.message = f"Servo {servo_number} has been set to angle {angle}"
            except Exception as e:
                response.success = False
                response.message = e
            return response
        else:
            response.success = False
            return("PCA9685 is not connected")
        
        


def main(args=None):
    rclpy.init(args=args)
    node = PCA9685DriverServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
