import time
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
        

def calib_angles(a):
    #left front leg
    kit.servo[1].angle = a
    kit.servo[3].angle = a
    kit.servo[5].angle = a
    time.sleep(1)
    #right front leg
    kit.servo[2].angle = a
    kit.servo[4].angle = a
    kit.servo[6].angle = a
    time.sleep(1)
    #left back leg
    kit.servo[15].angle = a
    kit.servo[13].angle = a
    kit.servo[11].angle = a
    time.sleep(1)
    #right back leg
    kit.servo[14].angle = a
    kit.servo[12].angle = a
    kit.servo[10].angle = a
    time.sleep(1)

def rest_mode():
    #left front leg
    kit.servo[1].angle = 15
    kit.servo[3].angle = 100
    kit.servo[5].angle = 45
    time.sleep(1)
    #right front leg
    kit.servo[2].angle = 30
    kit.servo[4].angle = 80
    kit.servo[6].angle = 135
    time.sleep(1)
    #left back leg
    kit.servo[15].angle = 0
    kit.servo[13].angle = 100
    kit.servo[11].angle = 40
    time.sleep(1)
    #right back leg
    kit.servo[14].angle = 30
    kit.servo[12].angle = 80
    kit.servo[10].angle = 135
    time.sleep(1)

def get_up():
    #shoulders
    kit.servo[5].angle = 100
    kit.servo[6].angle = 90
    kit.servo[10].angle = 100
    kit.servo[11].angle = 85
    time.sleep(0.5)
    #forearms
    kit.servo[3].angle = 30
    kit.servo[4].angle = 150
    time.sleep(0.5)
    kit.servo[1].angle = 95
    kit.servo[2].angle = 90
    time.sleep(0.5)
    kit.servo[13].angle = 40
    kit.servo[12].angle = 135
    time.sleep(0.5)
    kit.servo[15].angle = 90
    kit.servo[14].angle = 80
    time.sleep(1)


    time.sleep(1)

def lie_down():
    #arms
    kit.servo[1].angle = 15
    kit.servo[2].angle = 180
    kit.servo[15].angle = 0
    kit.servo[14].angle = 170
    time.sleep(1)
    #forearms
    kit.servo[3].angle = 100
    kit.servo[4].angle = 80
    kit.servo[13].angle = 100
    kit.servo[12].angle = 80
    time.sleep(1)
    #shoulders
    kit.servo[5].angle = 45
    kit.servo[6].angle = 140
    kit.servo[10].angle = 140
    kit.servo[11].angle = 40

def lie_down2():
    #arms
    kit.servo[15].angle = 0
    kit.servo[14].angle = 160
    time.sleep(1)
    kit.servo[1].angle = 15
    kit.servo[2].angle = 140
    time.sleep(1)
    #forearms
    kit.servo[3].angle = 100
    kit.servo[4].angle = 80
    kit.servo[13].angle = 100
    kit.servo[12].angle = 80
    time.sleep(1)
    #shoulders
    kit.servo[5].angle = 45
    kit.servo[6].angle = 140
    kit.servo[10].angle = 140
    kit.servo[11].angle = 40

def low(a):
    #shoulders
    kit.servo[5].angle = 100
    kit.servo[6].angle = 90
    kit.servo[10].angle = 100
    kit.servo[11].angle = 85
    time.sleep(1)
    #forearms
    kit.servo[3].angle = 80
    kit.servo[4].angle = 80
    kit.servo[13].angle = 85
    kit.servo[12].angle = 90
    time.sleep(1)
    #arms
    kit.servo[1].angle = 15
    kit.servo[2].angle = 180
    kit.servo[15].angle = 0
    kit.servo[14].angle = 170
    time.sleep(1)
    