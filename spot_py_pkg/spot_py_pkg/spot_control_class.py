import time
import numpy as np
import rclpy
from rclpy.node import Node

from adafruit_servokit import ServoKit

try:
    kit = ServoKit(channels=16)
except:
    print("ServoKit Not Connected, simulating command")
    class Servo:
        def __init__(self):
            self.angle = 0
            
    class Kit:
        def __init__(self):
            self.servo = [Servo() for _ in range(16)]
        
    kit = Kit()
    
class SpotPosition():
    def __init__(self):
        #Only to showcase their number
        self.servos_nb   = [ 1,   2,  3,  4,   5,  6,  7,  8,  9,  10, 11, 12,  13, 14, 15]
        #Default position, low position
        self.current_pos = [15, 180, 80, 80, 100, 90, -1, -1, -1, 100, 85, 90, 85, 170,  0]
        #maximum and minimum of the position
        self.min_pos = [ 15,  20,  20,  20,  20,  20, -1, -1, -1,  20,  20,  20,  20,  20,   0]
        self.max_pos = [160, 180, 160, 160, 160, 160, -1, -1, -1, 160, 160, 160, 160, 170, 160]

    def change_current_pos(self, servos, new_pos):
        for i, servo_nb in enumerate(servos):
            print(f"{i}, {servo_nb}, {new_pos[i]}")
            self.current_pos[servo_nb-1] = new_pos[i]
        print(self.current_pos)

    def get_current_pos(self, servos):
        current_pos = np.zeros(len(servos))
        for i, servo_nb in enumerate(servos):
            current_pos[i] = self.current_pos[servo_nb-1]
        print(current_pos)
        return current_pos
    
    def boundaries_check(self, servos, new_pos):
        if len(servos) != len(new_pos):
            print("length of servo array and position is different,cancelling movement")
            return 0
        counter = 0
        for i, servo_nb in enumerate(servos):
            #print(f"{i}, {servo_nb}, {new_pos[i]}")
            if (new_pos[i] < self.min_pos[servo_nb-1] and 
                new_pos[i] > self.max_pos[servo_nb-1]):
                print("position is out of boundaries, cancelling movement")
                self.get_logger().info("Robot Control has been Started")
                return 0
            if (new_pos[i] == self.current_pos[servo_nb-1]):
                counter+=1
        if counter == len(new_pos):
            print("position is the same as current, skipping movement")
            return 0
        return 1
    
    def controlled_motion(self, servos, new_positions, t):
        #checking validity of data
        position_valid = self.boundaries_check(servos, new_positions)
        if not position_valid:
            return
        #getting current pos
        current_pos = self.get_current_pos(servos)
        interval = 0.2 #[sec]
        num_intervals = (int)(t/interval)
        
        #making the matrix
        pos_matrix = np.zeros((len(new_positions), num_intervals))
        for i, pos in enumerate (new_positions):
            pos_matrix[i,:] = np.linspace(current_pos[i], pos, num_intervals)
        print(pos_matrix)
        
        #movement
        for j in range(num_intervals): 
            for i, servo_nb in enumerate(servos):
                print(f"servo {servo_nb} goes into position {pos_matrix[i,j]}")
                kit.servo[servo_nb] = pos_matrix[i,j]
            time.sleep(interval)
            
        #update current position
        self.change_current_pos(servos, new_positions)
        
    def low(self):
        #shoulders
        self.controlled_motion([5, 6, 10, 11], [100, 90, 100, 85], 2)
        #forearms
        self.controlled_motion([3, 4, 12 ,13], [80, 80, 90, 85], 2)
        #arms
        self.controlled_motion([1, 2, 14, 15], [15, 180, 170, 0], 2)
        
    def get_up(self):
        #shoulders
        self.controlled_motion([5, 6, 10, 11], [100, 90, 100, 85], 2)
        #forearms & arms
        self.controlled_motion([ 1,  2,  3,  4,  10,  11,  12, 13, 14, 15], 
                               [95, 90, 30, 150, 100, 85, 135, 40, 80, 90], 2)
        
    def lie_down(self):
        #shoulders
        self.controlled_motion([ 5,   6,  10, 11], 
                               [45, 140, 140, 40], 2)
        #forearms
        self.controlled_motion([  3,  4,  12 ,13], 
                               [100, 80, 80, 100], 2)
        #arms
        self.controlled_motion([ 1,   2,  14, 15], 
                               [15, 180, 170,  0], 2)
              
# spot_pos = SpotPosition()
# spot_pos.low()
# spot_pos.get_up()
# spot_pos.lie_down()
    
    