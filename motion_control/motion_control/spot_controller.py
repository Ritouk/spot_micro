import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from spot_interfaces.srv import SetServo, StateMachineInfo, StateMachineSet
from functools import partial
import time
import numpy as np

#from adafruit_servokit import ServoKit

from motion_control.defines import *


class SpotControllerNode(Node):
    def __init__(self):
        super().__init__("spot_controller")
        self.subscriber_ = self.create_subscription(
            String, "control_input", self.callback_keyboard_input, 10)
        self.get_logger().info(
            "Robot Control has been Started, initializing position")
        
        #init servo data
        self.data_init()
        
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
                
    def callback_keyboard_input(self, msg):
        self.get_logger().info(f"Control has been recieved: {msg.data}")
        self.switch(msg.data)
    
    def call_state_machine_info_server(self):
        client = self.create_client(StateMachineInfo, "state_machine_info")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Wairing for State machine server....")
            
        request = StateMachineInfo.Request()
        request.what_state = True
        
        origin = "spot_controller"
        future = client.call_async(request, origin)
        future.add_done_callback()
        
    def callback_state_machine_info_server(self, future, origin):
        try:
            response = future.result()
            self.machine_state = response.message
        except Exception as e:
            pass
        
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
            self.get_up(times=0.8)
        elif char == "l":
            self.get_logger().info("Low")
            self.low(times=0.8)
        elif char == "s":
            self.get_logger().info("Sit")
            self.sit(times = 1)
        elif char == "x":
            #NOT WORKING PROPERLY
            self.get_logger().info("Shutting down, putting robot in sleep")
            self.__del__()
        else:
            self.get_logger().info(f"No command exist for this input: {char}")
        self.get_logger().info(f"Ready for next Input")
    
    def call_servo_server(self, servo_nb, angle):
        client = self.create_client(SetServo, "set_servo")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for servo server....")
        request = SetServo.Request()
        request.servo_nb = servo_nb
        request.angle = angle
        
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_servo_server))
    
    def callback_call_servo_server(self, future):
        response = future.result()
        self.get_logger().info(response.message)
    
    def data_init(self):
        #Only to showcase their number
        self.servos_nb   = [ 1,   2,  3,  4,   5,  6,  7,  8,  9,  10, 11, 12,  13, 14, 15]
        
        #Default position, low position
        self.current_pos = DEFAULT_POS
        #maximum and minimum of the position
        self.min_pos = MIN_POS
        self.max_pos = MAX_POS
        
        self.machine_state = "init"
            
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
        self.call_servo_server(RFE, 90)
        self.call_servo_server(LFE, 90)
        self.call_servo_server(LBE, 90)
        self.call_servo_server(RBE, 90)
        time.sleep(1)
        #forearms
        self.call_servo_server(RFI, 90)
        self.call_servo_server(LFI, 90)
        self.call_servo_server(RBI, 90)
        self.call_servo_server(LBI, 90)
        #arms        
        self.call_servo_server(RFK, 90)
        self.call_servo_server(LFK, 90)
        self.call_servo_server(RBK, 90)
        self.call_servo_server(LBK, 90)
    
    def pos_test(self):
        #shoulders
        self.call_servo_server(RFE, TEST_POS[RFE])
        self.call_servo_server(LFE, TEST_POS[LFE])
        self.call_servo_server(LBE, TEST_POS[LBE])
        self.call_servo_server(RBE, TEST_POS[RBE])
        time.sleep(1)
        #forearms
        self.call_servo_server(RFI, TEST_POS[RFI])
        self.call_servo_server(LFI, TEST_POS[LFI])
        self.call_servo_server(RBI, TEST_POS[RBI])
        self.call_servo_server(LBI, TEST_POS[LBI])
        #arms        
        self.call_servo_server(RFK, TEST_POS[RFK])
        self.call_servo_server(LFK, TEST_POS[LFK])
        self.call_servo_server(RBK, TEST_POS[RBK])
        self.call_servo_server(LBK, TEST_POS[LBK])
        
    def direct_pos(self, array):
        for i in [RFE, LFE, RBE, LBE]:
            #print(f"servo {i} gets the value {array[i]}")
            self.call_servo_server(i, array[i])
        time.sleep(1)
        for i in [RFI, LFI, RBI, LBI, RFK, LFK, RBK, LBK]:
            #print(f"servo {i} gets the value {array[i]}")
            self.call_servo_server(i, array[i])
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
            
            
        
def main(args=None):
    rclpy.init(args=args)
    node = SpotControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
    


if __name__ == "__main__":
    main()
