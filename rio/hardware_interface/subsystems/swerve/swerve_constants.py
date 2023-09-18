import wpilib
from wpimath.geometry._geometry import *
import math

NAMESPACE = 'real'

ENABLE_2ND_ORDER = True

# Small Gear Should Face the back of the robot
# All wheel drive motors should not be inverted
# All axle turn motors should be inverted + sensor phase
# All Cancoders should be direction false
MODULE_CONFIG = {
    "front_left": {
        "wheel_joint_name": "front_left_wheel_joint",
        "wheel_motor_port": 3, #9
        "axle_joint_name": "front_left_axle_joint",
        "axle_motor_port": 1, #7
        "axle_encoder_port": 2, #8
        "encoder_offset": 18.721, # 248.203,
        "location" : Translation2d(-0.52085486, 0.52085486) # Translation2d(0.52085486, 0.52085486)
    },
    "front_right": {
        "wheel_joint_name": "front_right_wheel_joint",
        "wheel_motor_port": 6, #12
        "axle_joint_name": "front_right_axle_joint",
        "axle_motor_port": 4, #10
        "axle_encoder_port": 5, #11
        "encoder_offset": 45.439 + 180.0, #15.908, TODO: REDO ENCODER OFFSET
        "location" : Translation2d(-0.52085486, -0.52085486)# Translation2d(-0.52085486, 0.52085486)
    },
    "rear_left": {
        "wheel_joint_name": "rear_left_wheel_joint",
        "wheel_motor_port": 12, #6
        "axle_joint_name": "rear_left_axle_joint",
        "axle_motor_port": 10, #4
        "axle_encoder_port": 11, #5
        "encoder_offset": 16.084 + 180.0, #327.393, TODO: REDO ENCODER OFFSET
        "location" : Translation2d(0.52085486, 0.52085486) #Translation2d(0.52085486, -0.52085486)
    },
    "rear_right": {
        "wheel_joint_name": "rear_right_wheel_joint",
        "wheel_motor_port": 9, #3
        "axle_joint_name": "rear_right_axle_joint",
        "axle_motor_port": 7, #1
        "axle_encoder_port": 8, #2
        "encoder_offset": -9.141, #201.094,
        "location" : Translation2d(0.52085486, -0.52085486) # Translation2d(-0.52085486, -0.52085486)
    }
}

def getJointList():
    joint_list = []
    for module in MODULE_CONFIG.values():
        joint_list.append(module['wheel_joint_name'])
        joint_list.append(module['axle_joint_name'])
    return joint_list

AXLE_DIRECTION = False
WHEEL_DIRECTION = False
ENCODER_DIRECTION = True
WHEEL_JOINT_GEAR_RATIO = 6.75 #8.14
AXLE_JOINT_GEAR_RATIO = 150.0/7.0
TICKS_PER_REV = 2048.0
CMD_TIMEOUT_SECONDS = 1

nominal_voltage = 9.0
steer_current_limit = 20.0

# This is needed to fix bad float values being published by RTI from the RIO.
# To fix this, we scale the float and convert to integers. 
# Then we scale it back down inside the ROS2 hardware interface.
SCALING_FACTOR_FIX = 10000

# Encoder Constants
encoder_ticks_per_rev = 4096.0
encoder_reset_velocity = math.radians(0.5)
encoder_reset_iterations = 500

axle_pid_constants = {
    "kP": 0.7,
    "kI": 0.0,
    "kD": 0.1,
    "kF": 0.2,
    "kIzone": 0,
    "kPeakOutput": 1.0
}
# wheel_pid_constants = {
#     "kF": 1023.0/20660.0,
#     "kP": 0.1,
#     "kI": 0.001,
#     "kD": 5
# }
wheel_pid_constants = {
    "kF": 1023.0/20660.0,
    "kP": 0.1,
    "kI": 0,
    "kD": 0
}
slot_idx = 0
pid_loop_idx = 0
timeout_ms = 30

velocityConstant = 0.5
accelerationConstant = 0.25
# Conversion Functions
positionCoefficient = 2.0 * math.pi / TICKS_PER_REV / AXLE_JOINT_GEAR_RATIO
velocityCoefficient = positionCoefficient * 10.0
# axle (radians) -> shaft (ticks)
def getShaftTicks(radians, displacementType) -> int:
    if displacementType == "position":
        return int(radians / positionCoefficient)
    elif displacementType == "velocity":
        return int(radians / velocityCoefficient)
    else:
        return 0

# shaft (ticks) -> axle (radians)
def getAxleRadians(ticks, displacementType):
    if displacementType == "position":
        return ticks * positionCoefficient
    elif displacementType == "velocity":
        return ticks * velocityCoefficient
    else:
        return 0

wheelPositionCoefficient = (2.0 * math.pi) / TICKS_PER_REV / WHEEL_JOINT_GEAR_RATIO
wheelVelocityCoefficient = wheelPositionCoefficient * 10.0
# wheel (radians) -> shaft (ticks)
def getWheelShaftTicks(radians, displacementType) -> int:
    if displacementType == "position":
        return int(radians / wheelPositionCoefficient)
    elif displacementType == "velocity":
        return int(radians / wheelVelocityCoefficient)
    else:
        return 0

# shaft (ticks) -> wheel (radians)
def getWheelRadians(ticks, displacementType):
    if displacementType == "position":
        return ticks * wheelPositionCoefficient
    elif displacementType == "velocity":
        return ticks * wheelVelocityCoefficient
    else:
        return 0
    
def radiansToMeters(radians):
    wheel_rad = 0.0508
    return radians * wheel_rad

def metersToRadians(meters):
    wheel_rad = 0.0508
    return meters/wheel_rad