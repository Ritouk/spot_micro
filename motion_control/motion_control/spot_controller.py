import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

import time
import numpy as np

from adafruit_servokit import ServoKit

from motion_control.defines import *


class SpotControllerNode(Node):
    def __init__(self):
        super().__init__("spot_controller")
        self.subscriber_ = self.create_subscription(
            String, "control_input", self.callback_robot_news, 10)
        self.get_logger().info(
            "Robot Control has been Started, initializing position")
        #init Servo kit
        self.servokit_init()
        
        #init servo data
        self.servo_data_init()
        
        #intializing 
        if (IS_CALIB):
            print("Calibration Mode")
            self.direct_pos(CALIB_POS)
        elif (IS_POS_TEST):
            print("Position Testing")
            self.low_default()
        else:
            print("Waking up")
            self.wake_up()
            
        self.get_logger().info("Robot Ready for input")
               
    def __del__(self):
        print("Shutting down, putting robot in sleep")
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
            self.lie_down(times=1)
        elif char == "u":
            self.get_logger().info("Get Up")
            self.get_up(times=1)
        elif char == "l":
            self.get_logger().info("Low")
            self.low(times=1)
        elif char == "s":
            self.get_logger().info("Sit")
            self.sit(times = 0.5)
        elif char == "x":
            #NOT WORKING PROPERLY
            self.get_logger().info("Shutting down, putting robot in sleep")
            self.__del__()
        else:
            self.get_logger().info(f"No command exist for this input: {char}")
        self.get_logger().info(f"Ready for next Input")

    def servokit_init(self):
        try:
            self.kit = ServoKit(channels=16)
            print("ServoKit Connected!")
        except:
            print("ServoKit Not Connected, simulating command")
            class Servo:
                def __init__(self):
                    self.angle = 0
                    
            class Kit:
                def __init__(self):
                    self.servo = [Servo() for _ in range(16)]
                
            self.kit = Kit()
    
    def servo_data_init(self):
        #Only to showcase their number
        self.servos_nb   = [ 1,   2,  3,  4,   5,  6,  7,  8,  9,  10, 11, 12,  13, 14, 15]
        
        #Default position, low position
        self.current_pos = DEFAULT_POS
        #maximum and minimum of the position
        self.min_pos = MIN_POS
        self.max_pos = MAX_POS
            
    def change_current_pos(self, servos, new_pos):
        for i, servo_nb in enumerate(servos):
            #print(f"{i}, {servo_nb}, {new_pos[i]}")
            self.current_pos[servo_nb] = new_pos[i]
        print(self.current_pos)

    def get_current_pos(self, servos):
        current_pos = np.zeros(len(servos))
        for i, servo_nb in enumerate(servos):
            current_pos[i] = self.current_pos[servo_nb]
        #print(current_pos)
        return current_pos
    
    def boundaries_check(self, servos, new_pos):
        if len(servos) != len(new_pos):
            print("length of servo array and position is different,cancelling movement")
            return 0
        counter = 0
        for i, servo_nb in enumerate(servos):
            #print(f"{i}, {sdef angle_norm(self, servo, angle):ervo_nb}, {new_pos[i]}")
            if (new_pos[i] < MIN_POS[servo_nb] and 
                new_pos[i] > MAX_POS[servo_nb]):
                print("position is out of boundaries, cancelling movement")
                return 0
            if (new_pos[i] == self.current_pos[servo_nb]):
                counter+=1
        if counter == len(new_pos):
            print("position is the same as current, skipping movement")
            return 0
        return 1
    
    def controlled_motion_abs(self, servos, pos_array, t):
        #getting array for check
        new_positions = np.zeros(len((servos)))
        for i, servo_nb in enumerate(servos):
            new_positions[i] = pos_array[servo_nb]
            
        #checking validity of data
        position_valid = self.boundaries_check(servos, new_positions)
        if not position_valid:
            return
        
        #getting current pos
        current_pos = self.get_current_pos(servos)
        interval = 0.03 #[sec]
        num_intervals = (int)(t/interval)
        
        #making the matrix
        pos_matrix = np.zeros((len(new_positions), num_intervals))
        for i, pos in enumerate (new_positions):
            pos_matrix[i,:] = np.linspace(current_pos[i], pos, num_intervals)
        #print(pos_matrix)
        
        #movement
        for j in range(num_intervals): 
            for i, servo_nb in enumerate(servos):
                print(f"servo {servo_nb} goes into position {pos_matrix[i,j]}")
                self.kit.servo[servo_nb].angle = pos_matrix[i,j]
            time.sleep(interval)
            
        #update current position
        self.change_current_pos(servos, new_positions)
        
    def low(self, times):
        #shoulders
        self.controlled_motion_abs([RFE, LFE, LBE, RBE], LOW_POS, times)
        #forearms
        self.controlled_motion_abs([RFI,  LFI, LBI , RBI,  RFK,   LFK,  LBK, RBK], 
                                   LOW_POS, 
                                   times)
        
    def get_up(self, times):
        print("up! ")
        #shoulders
        self.controlled_motion_abs([RFE, LFE, LBE, RBE], 
                               UP_POS,
                               times)
        #forearms & arms
        self.controlled_motion_abs([ RFK, LFK, RFI, LFI, LBI, RBI, LBK, RBK], UP_POS, times)
    
    def lie_down(self, times):
        #shoulders
        self.controlled_motion_abs([RFE, LFE, LBE, RBE], 
                               DEFAULT_POS,
                               times)
        #forearms
        self.controlled_motion_abs([  RFI,  LFI,  LBI , RBI,  RFK,   LFK,  LBK, RBK], 
                               LIE_DOWN_POS, times)

        #shoulders
        self.controlled_motion_abs([ RFE,   LFE,  LBE, RBE], 
                               LIE_DOWN_POS, times)

    def sit(self, times):
        #shoulders
        self.controlled_motion_abs([RFE, LFE, LBE, RBE], 
                               SIT_POS
                               , times)
        #forearms
        self.controlled_motion_abs([ RFK, LFI, LBI , RBI,  RFE,  LFK,  LBK, RBK], 
                                    SIT_POS, times)
    
    def low_default(self):
        print("Getting to default - low")
        #shoulders
        self.kit.servo[RFE].angle = DEFAULT_POS[RFE]
        self.kit.servo[LFE].angle = DEFAULT_POS[LFE]
        self.kit.servo[LBE].angle = DEFAULT_POS[LBE]
        self.kit.servo[RBE].angle = DEFAULT_POS[RBE]
        time.sleep(1)
        #forearms
        self.kit.servo[RFI].angle = DEFAULT_POS[RFI]
        self.kit.servo[LFI].angle = DEFAULT_POS[LFI]
        self.kit.servo[RBI].angle = DEFAULT_POS[RBI]
        self.kit.servo[LBI].angle = DEFAULT_POS[LBI]
        #arms
        self.kit.servo[RFK].angle = DEFAULT_POS[RFK]
        self.kit.servo[LFK].angle = DEFAULT_POS[LFK]
        self.kit.servo[RBK].angle = DEFAULT_POS[RBK]
        self.kit.servo[LBK].angle = DEFAULT_POS[LBK]

        time.sleep(1)
        
    def wake_up(self):
        self.low_default()
        self.get_up(2)   
        
    def sleep(self):
        self.low(5)         

    def calib(self):
        #shoulders
        self.kit.servo[RFE].angle = 90
        self.kit.servo[LFE].angle = 90
        self.kit.servo[RBE].angle = 90
        self.kit.servo[LBE].angle = 90
        time.sleep(1)
        #forearms
        self.kit.servo[RFI].angle = 90
        self.kit.servo[LFI].angle = 90
        self.kit.servo[RBI].angle = 90
        self.kit.servo[LBI].angle = 90
        time.sleep(1)
        #arms
        self.kit.servo[RFK].angle = 90
        self.kit.servo[LFK].angle = 90
        self.kit.servo[RBK].angle = 90
        self.kit.servo[LBK].angle = 90
    
    def pos_test(self):
        #shoulders
        self.kit.servo[RFE].angle = TEST_POS[RFE]
        self.kit.servo[LFE].angle = TEST_POS[LFE]
        self.kit.servo[LBE].angle = TEST_POS[LBE]
        self.kit.servo[RBE].angle = TEST_POS[RBE]
        time.sleep(1)
        #forearms
        self.kit.servo[RFI].angle = TEST_POS[RFI]
        self.kit.servo[LFI].angle = TEST_POS[LFI]
        self.kit.servo[RBI].angle = TEST_POS[RBI]
        self.kit.servo[LBI].angle = TEST_POS[LBI]
        #arms
        self.kit.servo[RFK].angle = TEST_POS[RFK]
        self.kit.servo[LFK].angle = TEST_POS[LFK]
        self.kit.servo[RBK].angle = TEST_POS[RBK]
        self.kit.servo[LBK].angle = TEST_POS[LBK]
        
    def direct_pos(self, array):
        for i in [RFE, LFE, RBE, LBE]:
            #print(f"servo {i} gets the value {array[i]}")
            self.kit.servo[i].angle = array[i]
        time.sleep(1)
        for i in [RFI, LFI, RBI, LBI, RFK, LFK, RBK, LBK]:
            #print(f"servo {i} gets the value {array[i]}")
            self.kit.servo[i].angle = array[i]
        time.sleep(1)
                      
    def pos_test_safe(self, times):
        # getting to safe pos
        #self.low_default()
        #moving to position
        self.controlled_motion_abs([RFE, LFE, LBE, RBE], 
                               TEST_POS
                               , times)
        #forearms
        self.controlled_motion_abs([RFI, LFI, LBI, RBI, 
                                RFK, LFK, LBK, RBK], TEST_POS
                               , times)
            
    def angle_norm(self, servo, angle):
    #not sure its worth the headeache, will see.
        if (servo%0 !=0):
            self.kit.servo[servo].angle = 180 - angle
        else:
            self.kit.servo[servo].angle = angle
            
        
def main(args=None):
    rclpy.init(args=args)
    node = SpotControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
    


if __name__ == "__main__":
    main()