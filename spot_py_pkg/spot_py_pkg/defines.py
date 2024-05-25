##-- DEBUG MODE (1 yes, 0 no)
IS_CALIB = 0
IS_POS_TEST = 0
###------ Servo Nb
NIU = 0 #NOT IN USE
"""
XYZZ
X - L or R - Left or Right
Y - F or R - Front or Back (rear but easier to read)
ZZ - EL, IR, ER - Knee (Elbow but easier to read), Internal or External Rotation
"""
#Right Front Leg
RFK = 1
RFI = 3
RFE = 5
#Left Front Leg
LFK = 2
LFI = 4
LFE = 6
#Right Back Leg
RBK = 15
RBI = 13
RBE = 11
#Left Back Leg
LBK = 14
LBI = 12
LBE = 10

###-------- Servo data
SERVO_ORDER = [NIU, RFK, LFK, RFI, LFI, RFE, LFE, NIU, NIU, NIU, LBE, RBE, LBI, RBI, LBK, RBK]
# First Calibration, to assemble the servos 
CALIB_POS   = [NIU,  90,  90,  90,  90,  90,  90, NIU, NIU, NIU,  90,  90,  90,  90,  90,  90]
CALIB_CORR =  [NIU,  30, -30,   0,   0,   0,   0, NIU, NIU, NIU,   0,   0,   0,   0, -30,  30]
# Second Calibration, software correction for assemblies errors


# POS 0, 7, 8, 9 is not in use in the servo controller, so its replaced with a NIU 
SERVO_NB    = [NIU,  1,   2,  3,  4,   5,  6,  NIU, NIU, NIU, 10, 11, 12,  13, 14, 15]

# Absolute Positions
SERVO_ORDER  = [NIU, RFK, LFK, RFI, LFI, RFE, LFE, NIU, NIU, NIU, LBE, RBE, LBI, RBI, LBK, RBK]
DEFAULT_POS  = [NIU ,  0, 180,  90,  90,  90,  90, NIU, NIU, NIU,  90,  90,  90,  90, 180,   0] + CALIB_CORR
LOW_POS      = [NIU ,  0, 180,  90,  90,  90,  90, NIU, NIU, NIU,  90,  90,  90,  90, 180,   0] + CALIB_CORR
LIE_DOWN_POS = [NIU ,  0, 180,  90,  90,  45, 135, NIU, NIU, NIU, 135,  45,  90,  90, 180,   0] + CALIB_CORR

ALPHA = 45
UP_POS = [ a * ALPHA for a in [NIU, 1, -1, -1, 1, 0, 0, NIU, NIU, NIU, 0, 0, 1,-1, -1, 1]]
UP_POS = [a + b for a, b in zip(DEFAULT_POS, UP_POS)]

BETA = 45
SIT_POS = [ element * BETA for element in [NIU, 1, -1, -1, 1, 0, 0, NIU, NIU, NIU, 0, 1, -1, 0, 0, 0]]
SIT_POS = [a + b for a, b in zip(DEFAULT_POS, SIT_POS)]
#Relative Positions 

#This array is to test particular position, the servo will jump directly to the position, be careful! 
TEST_POS    = [NIU,  30, 150,  30, 150, 100,  90, NIU, NIU, NIU, 100,  85,  90,  85, 170,   0] + CALIB_CORR
#maximum and minimum of the position
MIN_POS =     [NIU,  20,  20,  20,  20,  20,  20, NIU, NIU, NIU,  20,  20,  20,  20,  20,  20]
MAX_POS =     [NIU, 160, 160, 160, 160, 160, 160, NIU, NIU, NIU, 160, 160, 160, 160, 160, 160]

MIN_POS =     [NIU,   0,   0,   0,   0,   0,   0, NIU, NIU, NIU,   0,   0,   0,   0,   0,   0]
MAX_POS =     [NIU, 180, 180, 180, 180, 180, 180, NIU, NIU, NIU, 180, 180, 180, 180, 180, 180]
"""
How to calibrate
1. Mechanical Calbiration
Use IS_CALIB = 1 to activate the calibration mode
All Servos will get to 90 degrees
Assemble the servos, trying to get as close to 90 degrees as possible.
2. Software Calibration
Use IS_POS_TEST = 1 to activate the calibration mode
External Rotation should be 90 degrees
Internal Rotation should be 90 degrees
Knees should be as entered as possible, 0 for right and 180 left

To correct errors, make changes in the CALIB_CORR array. 
In default, it should have enough margin to avoid any critical errors.

Change the values until you get the precision required

Update Min and Max pos to avoid any undesired angles, maximum or minimum
note: The calibration should of taken care of Left Knees Max and Right Knee min

"""