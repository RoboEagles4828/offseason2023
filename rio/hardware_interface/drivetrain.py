import wpilib
from wpimath.geometry._geometry import Translation2d, Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState
import ctre
import ctre.sensors
import math
import time
import logging
from wpimath.filter import SlewRateLimiter
from hardware_interface.joystick import Joystick
import navx
from hardware_interface.toggle import ToggleButton


NAMESPACE = 'real'

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
        "encoder_offset": 45.439 + 180.0, #15.908,
        "location" : Translation2d(-0.52085486, -0.52085486)# Translation2d(-0.52085486, 0.52085486)
    },
    "rear_left": {
        "wheel_joint_name": "rear_left_wheel_joint",
        "wheel_motor_port": 12, #6
        "axle_joint_name": "rear_left_axle_joint",
        "axle_motor_port": 10, #4
        "axle_encoder_port": 11, #5
        "encoder_offset": 16.084 + 180.0, #327.393,
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
    "kP": 0.2,
    "kI": 0.0,
    "kD": 0.1,
    "kF": 0.2,
    "kIzone": 0,
    "kPeakOutput": 1.0
}
wheel_pid_constants = {
        "kF": 1023.0/20660.0,
        "kP": 0.1,
        "kI": 0.001,
        "kD": 5
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

wheelPositionCoefficient = 2.0 * math.pi / TICKS_PER_REV / WHEEL_JOINT_GEAR_RATIO
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


class SwerveModule():
    def __init__(self, module_config) -> None:
        #IMPORTANT:
        # The wheel joint is the motor that drives the wheel.
        # The axle joint is the motor that steers the wheel.
        
        self.wheel_joint_name = module_config["wheel_joint_name"]
        self.axle_joint_name = module_config["axle_joint_name"]
        self.wheel_joint_port = module_config["wheel_motor_port"]
        self.axle_joint_port = module_config["axle_motor_port"]
        self.axle_encoder_port = module_config["axle_encoder_port"]
        self.encoder_offset = module_config["encoder_offset"]

        self.wheel_motor = ctre.WPI_TalonFX(self.wheel_joint_port, "rio")
        self.axle_motor = ctre.WPI_TalonFX(self.axle_joint_port, "rio")
        self.encoder = ctre.sensors.WPI_CANCoder(self.axle_encoder_port, "rio")

        self.last_wheel_vel_cmd = None
        self.last_axle_vel_cmd = None
        self.reset_iterations = 0
        
        self.setupEncoder()
        self.setupWheelMotor()
        self.setupAxleMotor()
        

    def setupEncoder(self):
        self.encoderconfig = ctre.sensors.CANCoderConfiguration()
        self.encoderconfig.absoluteSensorRange = ctre.sensors.AbsoluteSensorRange.Unsigned_0_to_360
        self.encoderconfig.initializationStrategy = ctre.sensors.SensorInitializationStrategy.BootToAbsolutePosition
        self.encoderconfig.magnetOffsetDegrees = self.encoder_offset
        self.encoderconfig.sensorDirection = ENCODER_DIRECTION
        self.encoder.configAllSettings(self.encoderconfig)
        self.encoder.setPositionToAbsolute(timeout_ms)
        self.encoder.setStatusFramePeriod(ctre.sensors.CANCoderStatusFrame.SensorData, 10, timeout_ms)
    
    def getEncoderPosition(self):
        return math.radians(self.encoder.getAbsolutePosition())
    
    def getEncoderVelocity(self):
        return math.radians(self.encoder.getVelocity())
    
    def setupWheelMotor(self):
        self.wheel_motor.configFactoryDefault()
        self.wheel_motor.configNeutralDeadband(0.01, timeout_ms)

        # Direction and Sensors
        self.wheel_motor.setSensorPhase(WHEEL_DIRECTION)
        self.wheel_motor.setInverted(WHEEL_DIRECTION)
        self.wheel_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor, slot_idx, timeout_ms)
        self.wheel_motor.setStatusFramePeriod(ctre.StatusFrameEnhanced.Status_21_FeedbackIntegrated, 10, timeout_ms)
        
        # Peak and Nominal Outputs
        self.wheel_motor.configNominalOutputForward(0, timeout_ms)
        self.wheel_motor.configNominalOutputReverse(0, timeout_ms)
        self.wheel_motor.configPeakOutputForward(1, timeout_ms)
        self.wheel_motor.configPeakOutputReverse(-1, timeout_ms)

        # Voltage Comp
        self.wheel_motor.configVoltageCompSaturation(nominal_voltage, timeout_ms)
        self.axle_motor.enableVoltageCompensation(True)

        # Tuning
        self.wheel_motor.config_kF(0, wheel_pid_constants["kF"], timeout_ms)
        self.wheel_motor.config_kP(0, wheel_pid_constants["kP"], timeout_ms)
        self.wheel_motor.config_kI(0, wheel_pid_constants["kI"], timeout_ms)
        self.wheel_motor.config_kD(0, wheel_pid_constants["kD"], timeout_ms)
        
        # Brake
        self.wheel_motor.setNeutralMode(ctre.NeutralMode.Brake)

        # Velocity Ramp
        # TODO: Tweak this value

        # Current Limit
        current_limit = 20
        current_threshold = 40
        current_threshold_time = 0.1
        supply_current_limit_configs = ctre.SupplyCurrentLimitConfiguration(True, current_limit, current_threshold, current_threshold_time)
        self.wheel_motor.configSupplyCurrentLimit(supply_current_limit_configs, timeout_ms)

    
    def setupAxleMotor(self):
        self.axle_motor.configFactoryDefault()
        self.axle_motor.configNeutralDeadband(0.01, timeout_ms)
        
        # Direction and Sensors
        self.axle_motor.setSensorPhase(AXLE_DIRECTION)
        self.axle_motor.setInverted(AXLE_DIRECTION)
        self.axle_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor, slot_idx, timeout_ms)
        # self.axle_motor.setStatusFramePeriod(ctre.StatusFrameEnhanced.Status_13_Base_PIDF0, 10, timeout_ms)
        self.axle_motor.setStatusFramePeriod(ctre.StatusFrameEnhanced.Status_10_MotionMagic, 10, timeout_ms)
        self.axle_motor.setSelectedSensorPosition(getShaftTicks(self.getEncoderPosition(), "position"), pid_loop_idx, timeout_ms)

        # Peak and Nominal Outputs
        self.axle_motor.configNominalOutputForward(0, timeout_ms)
        self.axle_motor.configNominalOutputReverse(0, timeout_ms)
        self.axle_motor.configPeakOutputForward(1, timeout_ms)
        self.axle_motor.configPeakOutputReverse(-1, timeout_ms)

        # Tuning
        self.axle_motor.selectProfileSlot(slot_idx, pid_loop_idx)
        self.axle_motor.config_kP(slot_idx, axle_pid_constants["kP"], timeout_ms)
        self.axle_motor.config_kI(slot_idx, axle_pid_constants["kI"], timeout_ms)
        self.axle_motor.config_kD(slot_idx, axle_pid_constants["kD"], timeout_ms)
        self.axle_motor.config_kF(slot_idx, (1023.0 *  velocityCoefficient / nominal_voltage) * velocityConstant, timeout_ms)
        self.axle_motor.configMotionCruiseVelocity(2.0 / velocityConstant / velocityCoefficient, timeout_ms)
        self.axle_motor.configMotionAcceleration((8.0 - 2.0) / accelerationConstant / velocityCoefficient, timeout_ms)
        self.axle_motor.configMotionSCurveStrength(2)

        # Voltage Comp
        self.axle_motor.configVoltageCompSaturation(nominal_voltage, timeout_ms)
        self.axle_motor.enableVoltageCompensation(True)

        # Braking
        self.axle_motor.setNeutralMode(ctre.NeutralMode.Brake)

    def neutralize_module(self):
        self.wheel_motor.set(ctre.TalonFXControlMode.PercentOutput, 0)
        self.axle_motor.set(ctre.TalonFXControlMode.PercentOutput, 0)

    def set(self, state: SwerveModuleState):
        wheel_radius = 0.0508
        set_wheel_motor_vel = state.speed / (wheel_radius) / math.pi * 2
        self.wheel_motor.set(ctre.TalonFXControlMode.Velocity, set_wheel_motor_vel)
        self.axle_motor.set(ctre.TalonFXControlMode.MotionMagic, getShaftTicks(state.angle.radians(), "position"))

    def getEncoderData(self):
        output = [
            {
                "name": self.wheel_joint_name,
                "position": int(getWheelRadians(self.wheel_motor.getSelectedSensorPosition(), "position") * SCALING_FACTOR_FIX),
                "velocity": int(getWheelRadians(self.wheel_motor.getSelectedSensorVelocity(), "velocity") * SCALING_FACTOR_FIX)
            },
            {
                "name": self.axle_joint_name,
                "position": int(self.getEncoderPosition() * SCALING_FACTOR_FIX),
                "velocity": int(self.getEncoderVelocity() * SCALING_FACTOR_FIX)
            }
        ]
        return output
    
############################################################################################################################################################
# DriveTrain Class
class DriveTrain():
    def __init__(self):
        self.last_cmds = { "name" : getJointList(), "position": [0.0]*len(getJointList()), "velocity": [0.0]*len(getJointList()) }
        self.last_cmds_time = time.time()
        self.warn_timeout = True
        self.front_left = SwerveModule(MODULE_CONFIG["front_left"])
        self.front_right = SwerveModule(MODULE_CONFIG["front_right"])
        self.rear_left = SwerveModule(MODULE_CONFIG["rear_left"])
        self.rear_right = SwerveModule(MODULE_CONFIG["rear_right"])
        self.front_left_location = MODULE_CONFIG["front_left"]["location"]
        self.front_right_location = MODULE_CONFIG["front_right"]["location"]
        self.rear_left_location = MODULE_CONFIG["rear_left"]["location"]
        self.rear_right_location = MODULE_CONFIG["rear_right"]["location"]
        self.kinematics = SwerveDrive4Kinematics(self.front_left_location, self.front_right_location, self.rear_left_location, self.rear_right_location)
        self.navx = navx.AHRS.create_spi()
        self.navx.calibrate()
        self.speed = ChassisSpeeds(0, 0, 0)
        self.wheel_radius = 0.0508

        self.ROBOT_MAX_TRANSLATIONAL = 250 #16.4041995 # 5.0 # m/s
        self.ROBOT_MAX_ROTATIONAL = 250 * math.pi #16.4041995 * math.pi #rad/s

        self.MODULE_MAX_SPEED = 50 #16.4041995 # m/s

        self.move_scale = self.ROBOT_MAX_TRANSLATIONAL
        self.turn_scale = self.ROBOT_MAX_ROTATIONAL

        self.slew_X = SlewRateLimiter(0.5)
        self.slew_Y = SlewRateLimiter(0.5)
        self.slew_Z = SlewRateLimiter(0.5)

        self.slew_X.reset(0)
        self.slew_Y.reset(0)
        self.slew_Z.reset(0)

        self.field_oriented_button = ToggleButton(7, False)
        self.field_oriented_value = False

        self.module_lookup = \
        {
            'front_left_axle_joint': self.front_left,
            'front_right_axle_joint': self.front_right,
            'rear_left_axle_joint': self.rear_left,
            'rear_right_axle_joint': self.rear_right,
        }
        self.toggle = False

    def reset_slew(self):
        self.slew_X.reset(0)
        self.slew_Y.reset(0)
        self.slew_Z.reset(0)

    def getEncoderData(self):
        names = [""]*8
        positions = [0]*8
        velocities = [0]*8

        encoderInfo = []
        encoderInfo += self.front_left.getEncoderData() 
        encoderInfo += self.front_right.getEncoderData()
        encoderInfo += self.rear_left.getEncoderData()
        encoderInfo += self.rear_right.getEncoderData()
        assert len(encoderInfo) == 8
        for index, encoder in enumerate(encoderInfo):
            names[index] = encoder['name']
            positions[index] = encoder['position']
            velocities[index] = encoder['velocity']
        return { "name": names, "position": positions, "velocity": velocities }

    def stop(self):
        self.front_left.neutralize_module()
        self.front_right.neutralize_module()
        self.rear_left.neutralize_module()
        self.rear_right.neutralize_module()

    def testModule(self):
        self.rear_right.encoderOffsetTest()

    def swerveDrive(self, joystick: Joystick):      
        # slew 
        # gives joystick ramping
        linearX = self.slew_X.calculate(joystick.getData()["axes"][1]) * self.move_scale
        linearY = self.slew_Y.calculate(joystick.getData()["axes"][0]) * -self.move_scale
        angularZ = self.slew_Z.calculate(joystick.getData()["axes"][3]) * self.turn_scale

        if joystick.getData()["buttons"][7] == 1.0:
            self.field_oriented_value = self.field_oriented_button.toggle(joystick.getData()["buttons"])
        else:
            self.field_oriented_value = self.field_oriented_button.toggle(joystick.getData()["buttons"])

        if joystick.getData()["buttons"][6] == 1.0:
            self.navx.zeroYaw()

        if abs(joystick.getData()["axes"][5]) >= 0.5 :
            self.move_scale = self.ROBOT_MAX_TRANSLATIONAL / 2.0
            self.turn_scale = self.ROBOT_MAX_ROTATIONAL / 4.0
        else:
            self.move_scale = self.ROBOT_MAX_TRANSLATIONAL
            self.move_scale = self.ROBOT_MAX_ROTATIONAL

        if self.field_oriented_value:
            print(f"NavX: {self.navx.getRotation2d().degrees()}", end=" ")
            # field  oriented
            self.speeds = ChassisSpeeds.fromFieldRelativeSpeeds(linearY, linearX, angularZ, self.navx.getRotation2d().rotateBy(Rotation2d(math.radians(-90))))
        else:
            self.speeds = ChassisSpeeds(linearX, linearY, angularZ)

        self.module_state = self.kinematics.toSwerveModuleStates(self.speeds)
        
        # normalize speeds
        # if the speeds are greater than the max speed, scale them down
        self.kinematics.desaturateWheelSpeeds(self.module_state, self.speeds, self.MODULE_MAX_SPEED, self.ROBOT_MAX_TRANSLATIONAL, self.ROBOT_MAX_ROTATIONAL)

        self.front_left_state: SwerveModuleState = self.module_state[0]
        self.front_right_state: SwerveModuleState = self.module_state[1]
        self.rear_left_state: SwerveModuleState = self.module_state[2]
        self.rear_right_state: SwerveModuleState = self.module_state[3]

        # optimize states
        # This makes the modules take the shortest path to the desired angle
        self.front_left_state = SwerveModuleState.optimize(self.front_left_state, Rotation2d(self.front_left.getEncoderPosition()))
        self.front_right_state = SwerveModuleState.optimize(self.front_right_state, Rotation2d(self.front_right.getEncoderPosition()))
        self.rear_left_state = SwerveModuleState.optimize(self.rear_left_state, Rotation2d(self.rear_left.getEncoderPosition()))
        self.rear_right_state = SwerveModuleState.optimize(self.rear_right_state, Rotation2d(self.rear_right.getEncoderPosition()))

        # print encoder postions
        # print("\nEncoder Positions")
        # print(f"Front Left: {self.front_left.getEncoderPosition()}")
        # print(f"Front Right: {self.front_right.getEncoderPosition()}")
        # print(f"Rear Left: {self.rear_left.getEncoderPosition()}")
        # print(f"Rear Right: {self.rear_right.getEncoderPosition()}")

        # print states
        # print(f"Front Left: {round(self.front_left_state.speed, 2)} {round(self.front_left_state.angle.degrees(), 2)}", end=" ")
        # print(f"Front Right: {round(self.front_right_state.speed, 2)} {round(self.front_right_state.angle.degrees(), 2)}", end=" ")
        # print(f"Rear Left: {round(self.rear_left_state.speed, 2)} {round(self.rear_left_state.angle.degrees(), 2)}", end=" ")
        # print(f"Rear Right: {round(self.rear_right_state.speed, 2)} {round(self.rear_right_state.angle.degrees(), 2)}")

        print(f"{linearX} {linearY} {angularZ}")
        # if abs(linearX) <= 3 and abs(linearY) <= 3 and abs(angularZ) <= 3:
        #     self.stop()
        # else:
        self.front_left.set(self.front_left_state)
        self.front_right.set(self.front_right_state)
        self.rear_left.set(self.rear_left_state)
        self.rear_right.set(self.rear_right_state)

    def swerveDriveAuton(self, linearX, linearY, angularZ):
        self.speeds = ChassisSpeeds(linearX, linearY, angularZ)

        self.module_state = self.kinematics.toSwerveModuleStates(self.speeds)
        self.kinematics.desaturateWheelSpeeds(self.module_state, self.speeds, self.MODULE_MAX_SPEED, self.ROBOT_MAX_TRANSLATIONAL, self.ROBOT_MAX_ROTATIONAL)

        self.front_left_state: SwerveModuleState = self.module_state[0]
        self.front_right_state: SwerveModuleState = self.module_state[1]
        self.rear_left_state: SwerveModuleState = self.module_state[2]
        self.rear_right_state: SwerveModuleState = self.module_state[3]

        # optimize states
        self.front_left_state = SwerveModuleState.optimize(self.front_left_state, Rotation2d(self.front_left.getEncoderPosition()))
        self.front_right_state = SwerveModuleState.optimize(self.front_right_state, Rotation2d(self.front_right.getEncoderPosition()))
        self.rear_left_state = SwerveModuleState.optimize(self.rear_left_state, Rotation2d(self.rear_left.getEncoderPosition()))
        self.rear_right_state = SwerveModuleState.optimize(self.rear_right_state, Rotation2d(self.rear_right.getEncoderPosition()))

        # # print encoder postions
        # print("\nEncoder Positions")
        # print(f"Front Left: {self.front_left.getEncoderPosition()}")
        # print(f"Front Right: {self.front_right.getEncoderPosition()}")
        # print(f"Rear Left: {self.rear_left.getEncoderPosition()}")
        # print(f"Rear Right: {self.rear_right.getEncoderPosition()}")

        # # print states
        # print("States")
        # print(f"Front Left: {self.front_left_state.speed} {self.front_left_state.angle.degrees()}")
        # print(f"Front Right: {self.front_right_state.speed} {self.front_right_state.angle.degrees()}")
        # print(f"Rear Left: {self.rear_left_state.speed} {self.rear_left_state.angle.degrees()}")
        # print(f"Rear Right: {self.rear_right_state.speed} {self.rear_right_state.angle.degrees()}")

        self.front_left.set(self.front_left_state)
        self.front_right.set(self.front_right_state)
        self.rear_left.set(self.rear_left_state)
        self.rear_right.set(self.rear_right_state)




