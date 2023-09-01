import wpilib
from wpimath.geometry._geometry import *
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState
from wpimath.controller import ProfiledPIDControllerRadians
from wpimath._controls._controls.trajectory import TrapezoidProfileRadians
from hardware_interface.motion_magic import MotionMagic
from hardware_interface import GeometryUtils
import ctre
import ctre.sensors
import math
import time
import logging
from wpimath.filter import SlewRateLimiter
from hardware_interface.joystick import Joystick
import navx
from hardware_interface.toggle import ToggleButton
from hardware_interface.navxSim import NavxSim


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
        
        self.neutralize_count = 0
        
        self.setupEncoder()
        self.setupWheelMotor()
        self.setupAxleMotor()
        

    def setupEncoder(self):
        self.encoderconfig = ctre.sensors.CANCoderConfiguration()
        self.encoderconfig.absoluteSensorRange = ctre.sensors.AbsoluteSensorRange.Unsigned_0_to_360
        self.encoderconfig.initializationStrategy = ctre.sensors.SensorInitializationStrategy.BootToAbsolutePosition
        self.encoder.setStatusFramePeriod(ctre.sensors.CANCoderStatusFrame.SensorData, 10, timeout_ms)
        
    def getMotorPosition(self):
        return getAxleRadians(self.axle_motor.getSelectedSensorPosition(), 'position')
    
    def getEncoderPosition(self):
        return math.radians(self.encoder.getPosition())
    
    def getEncoderVelocity(self):
        return math.radians(self.encoder.getVelocity())
    
    def setupWheelMotor(self):
        self.wheel_motor.configFactoryDefault(timeout_ms)
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

        # Supply Current Limit
        supply_current_limit = 20
        supply_current_threshold = 40
        supply_current_threshold_time = 0.1
        supply_current_limit_configs = ctre.SupplyCurrentLimitConfiguration(True, supply_current_limit, supply_current_threshold, supply_current_threshold_time)
        self.wheel_motor.configSupplyCurrentLimit(supply_current_limit_configs, timeout_ms)
        
        # Stator Current Limit
        stator_current_limit = 20
        stator_current_threshold = 40
        stator_current_threshold_time = 0.1
        stator_current_limit_configs = ctre.StatorCurrentLimitConfiguration(True, stator_current_limit, stator_current_threshold, stator_current_threshold_time)
        self.wheel_motor.configStatorCurrentLimit(stator_current_limit_configs, timeout_ms)

        # Velocity Ramp
        self.wheel_motor.configClosedloopRamp(0)
        self.wheel_motor.configOpenloopRamp(0)

    
    def setupAxleMotor(self):
        self.axle_motor.configFactoryDefault(timeout_ms)
        self.axle_motor.configNeutralDeadband(0.01, timeout_ms)
        
        # Direction and Sensors
        self.axle_motor.setSensorPhase(AXLE_DIRECTION)
        self.axle_motor.setInverted(AXLE_DIRECTION)
        self.axle_motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor, slot_idx, timeout_ms)
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

        # Velocity Ramp Removed
        self.axle_motor.configClosedloopRamp(0)
        self.axle_motor.configOpenloopRamp(0)

        # Supply Current Limit
        supply_current_limit = 20
        supply_current_threshold = 40
        supply_current_threshold_time = 0.1
        supply_current_limit_configs = ctre.SupplyCurrentLimitConfiguration(True, supply_current_limit, supply_current_threshold, supply_current_threshold_time)
        self.axle_motor.configSupplyCurrentLimit(supply_current_limit_configs, timeout_ms)

        # Stator Current Limit
        stator_current_limit = 20
        stator_current_threshold = 40
        stator_current_threshold_time = 0.1
        stator_current_limit_configs = ctre.StatorCurrentLimitConfiguration(True, stator_current_limit, stator_current_threshold, stator_current_threshold_time)
        self.axle_motor.configStatorCurrentLimit(stator_current_limit_configs, timeout_ms)

    def neutralize_module(self):
        self.wheel_motor.set(ctre.TalonFXControlMode.PercentOutput, 0)
        self.axle_motor.set(ctre.TalonFXControlMode.PercentOutput, 0)
        
    def neutralize_wheel(self):
        self.wheel_motor.set(ctre.TalonFXControlMode.PercentOutput, 0)
        
    def setMotors(self, wheel_motor_vel, axle_position):
        # WHEEL VELOCITY CONTROL
        wheel_vel = getWheelShaftTicks(wheel_motor_vel, "velocity")                                                                                            
        self.wheel_motor.set(ctre.TalonFXControlMode.Velocity, wheel_vel)
        self.last_wheel_vel_cmd = wheel_vel

        # MOTION MAGIC CONTROL FOR AXLE POSITION
        axle_motorPosition = getAxleRadians(self.axle_motor.getSelectedSensorPosition(), "position")
        axle_motorVelocity = getAxleRadians(self.axle_motor.getSelectedSensorVelocity(), "velocity")
        axle_absolutePosition = self.getEncoderPosition()

        # Reset
        if axle_motorVelocity < encoder_reset_velocity:
            self.reset_iterations += 1
            if self.reset_iterations >= encoder_reset_iterations:
                self.reset_iterations = 0
                self.axle_motor.setSelectedSensorPosition(getShaftTicks(axle_absolutePosition, "position"))
                axle_motorPosition = axle_absolutePosition
        else:
            self.reset_iterations = 0

        # First let's assume that we will move directly to the target position.
        newAxlePosition = axle_position

        # The motor could get to the target position by moving clockwise or counterclockwise.
        # The shortest path should be the direction that is less than pi radians away from the current motor position.
        # The shortest path could loop around the circle and be less than 0 or greater than 2pi.
        # We need to get the absolute current position to determine if we need to loop around the 0 - 2pi range.
        
        # The current motor position does not stay inside the 0 - 2pi range.
        # We need the absolute position to compare with the target position.
        axle_absoluteMotorPosition = math.fmod(axle_motorPosition, 2.0 * math.pi)
        if axle_absoluteMotorPosition < 0.0:
            axle_absoluteMotorPosition += 2.0 * math.pi

        # If the target position was in the first quadrant area 
        # and absolute motor position was in the last quadrant area
        # then we need to move into the next loop around the circle.
        if newAxlePosition - axle_absoluteMotorPosition < -math.pi:
            newAxlePosition += 2.0 * math.pi
        # If the target position was in the last quadrant area
        # and absolute motor position was in the first quadrant area
        # then we need to move into the previous loop around the circle.
        elif newAxlePosition - axle_absoluteMotorPosition > math.pi:
            newAxlePosition -= 2.0 * math.pi

        # Last, add the current existing loops that the motor has gone through.
        newAxlePosition += axle_motorPosition - axle_absoluteMotorPosition
        self.axle_motor.set(ctre.TalonFXControlMode.MotionMagic, getShaftTicks(newAxlePosition, "position"))
        #logging.info(f'{self.wheel_joint_name} {newAxlePosition} {wheel_motor_vel}')
        # logging.info('WHEEL MOTOR VEL: ', wheel_vel)

    def set(self, state: SwerveModuleState):
        set_wheel_motor_vel = metersToRadians(state.speed)
        set_axle_motor_pos = state.angle.radians()
        self.setMotors(set_wheel_motor_vel, set_axle_motor_pos)

    def setVelocity(self, velocity):
        set_wheel_motor_vel = metersToRadians(velocity)
        self.wheel_motor.set(ctre.TalonFXControlMode.Velocity, getWheelShaftTicks(set_wheel_motor_vel, "velocity"))

    def getEncoderData(self):
        output = [
            {
                "name": self.wheel_joint_name,
                "position": int(getWheelRadians(self.wheel_motor.getSelectedSensorPosition(), "position") * SCALING_FACTOR_FIX),
                "velocity": int(getWheelRadians(self.wheel_motor.getSelectedSensorVelocity(), "velocity") * SCALING_FACTOR_FIX),
                "effort": self.getWheelMotorEffort()
            },
            {
                "name": self.axle_joint_name,
                "position": int(self.getEncoderPosition() * SCALING_FACTOR_FIX),
                "velocity": int(self.getEncoderVelocity() * SCALING_FACTOR_FIX),
                "effort": self.getAxleMotorEffort()
            }
        ]
        return output
    
    def getAxleMotorEffort(self):
        output = self.axle_motor.getMotorOutputPercent()
        output = round(output/0.17, 2)
        
        if abs(output) < 1.0:
            return int(output * SCALING_FACTOR_FIX)
        else:
            if output < 0:
                return int(-SCALING_FACTOR_FIX)
            else:
                return int(SCALING_FACTOR_FIX)
        
    def getWheelMotorEffort(self):
        output = self.wheel_motor.getMotorOutputPercent()
        output = round(output/0.17, 2)
        
        if abs(output) < 1.0:
            return int(output * SCALING_FACTOR_FIX)
        else:
            if output < 0:
                return int(-SCALING_FACTOR_FIX)
            else:
                return int(SCALING_FACTOR_FIX)
    
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
        self.navx_sim = NavxSim()
        self.navx.calibrate()
        self.speeds = ChassisSpeeds(0, 0, 0)
        self.wheel_radius = 0.0508

        self.ROBOT_MAX_TRANSLATIONAL = 5.0 #16.4041995 # 5.0 # m/s
        self.ROBOT_MAX_ROTATIONAL = 5.0 #16.4041995 * math.pi #rad/s

        self.MODULE_MAX_SPEED = 5.0 #16.4041995 # m/s

        self.move_scale_x = self.ROBOT_MAX_TRANSLATIONAL
        self.move_scale_y = self.ROBOT_MAX_TRANSLATIONAL
        self.turn_scale = self.ROBOT_MAX_ROTATIONAL

        self.slew_X = SlewRateLimiter(2)
        self.slew_Y = SlewRateLimiter(2)
        self.slew_Z = SlewRateLimiter(2)
        self.slew_slow_translation = SlewRateLimiter(0.5)
        self.slew_slow_rotation = SlewRateLimiter(0.5)

        self.slew_X.reset(0)
        self.slew_Y.reset(0)
        self.slew_Z.reset(0)

        self.field_oriented_button = ToggleButton(7, False)
        self.field_oriented_value = False

        self.auto_turn_value = "off"

        self.profile_selector = wpilib.SendableChooser()
        self.profile_selector.setDefaultOption("Competition", (5.0, 5.0))
        self.profile_selector.addOption("Workshop", (2.5, 1.25))

        self.whine_remove_selector = wpilib.SendableChooser()
        self.whine_remove_selector.setDefaultOption("OFF", False)
        self.whine_remove_selector.addOption("ON", True)
        
        self.angle_source_selector = wpilib.SendableChooser()
        self.angle_source_selector.addOption("Angle", "angle")
        self.angle_source_selector.addOption("Yaw", "yaw")
        self.angle_source_selector.setDefaultOption("Normal", "normal")
        

        self.slow = False

        self.module_lookup = \
        {
            'front_left_axle_joint': self.front_left,
            'front_right_axle_joint': self.front_right,
            'rear_left_axle_joint': self.rear_left,
            'rear_right_axle_joint': self.rear_right,
        }
        self.last_print = ""

        self.last_state = ChassisSpeeds(0, 0, 0)

        self.motor_temps = []
        
        self.linX = 0
        self.linY = 0
        self.angZ = 0
        
        self.motor_vels = []
        self.motor_pos = []
        
        real_mm_accel = (8.0 - 2.0) / accelerationConstant / velocityCoefficient
        real_mm_vel = 2.0 / velocityConstant / velocityCoefficient
        
        self.new_motion_magic_1 = MotionMagic(getAxleRadians(real_mm_accel, "velocity") * 10, getAxleRadians(real_mm_vel, "velocity"))
        self.new_motion_magic_2 = MotionMagic(getAxleRadians(real_mm_accel, "velocity") * 10, getAxleRadians(real_mm_vel, "velocity"))
        self.new_motion_magic_3 = MotionMagic(getAxleRadians(real_mm_accel, "velocity") * 10, getAxleRadians(real_mm_vel, "velocity"))
        self.new_motion_magic_4 = MotionMagic(getAxleRadians(real_mm_accel, "velocity") * 10, getAxleRadians(real_mm_vel, "velocity"))
        
        self.front_left_state = SwerveModuleState(0, Rotation2d(0))
        self.front_right_state = SwerveModuleState(0, Rotation2d(0))
        self.rear_left_state = SwerveModuleState(0, Rotation2d(0))
        self.rear_right_state = SwerveModuleState(0, Rotation2d(0))
        
        self.is_sim = False
        self.locked = False
        
    def reset_slew(self):
        self.slew_X.reset(0)
        self.slew_Y.reset(0)
        self.slew_Z.reset(0)

    def getEncoderData(self):
        names = [""]*8
        positions = [0]*8
        velocities = [0]*8
        efforts = [0]*8

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
            efforts[index] = encoder['effort']
        return { "name": names, "position": positions, "velocity": velocities, "effort": efforts }

    def stop(self):
        self.front_left.neutralize_module()
        self.front_right.neutralize_module()
        self.rear_left.neutralize_module()
        self.rear_right.neutralize_module()

    def testModule(self):
        self.rear_right.encoderOffsetTest()

    def allZero(self, speeds: ChassisSpeeds):
        if 0 <= abs(speeds.vx) <= 0.1:
            if 0 <= abs(speeds.vy) <= 0.1:
                if 0 <= abs(speeds.omega) <= 0.1:
                    return True
        return False
    
    def customOptimize(self, desiredState: SwerveModuleState, currentAngle: Rotation2d):
        delta = desiredState.angle.__sub__(currentAngle)
        if abs(delta.degrees()) > 90:
            return SwerveModuleState(desiredState.speed * -1.0, desiredState.angle.rotateBy(Rotation2d.fromDegrees(180)))
        else:
            return SwerveModuleState(desiredState.speed, desiredState.angle)
        
    def correctForDynamics(self, originalSpeeds: ChassisSpeeds):
        LOOP_TIME_S = 0.02
        futureRobotPose = Pose2d(
            originalSpeeds.vx * LOOP_TIME_S,
            originalSpeeds.vy * LOOP_TIME_S,
            Rotation2d(originalSpeeds.omega * LOOP_TIME_S)
        )
        twistForPose = GeometryUtils.log(futureRobotPose)
        updatedSpeeds = ChassisSpeeds(
            twistForPose.dx / LOOP_TIME_S,
            twistForPose.dy / LOOP_TIME_S,
            twistForPose.dtheta / LOOP_TIME_S
        )
        return updatedSpeeds

    def swerveDrive(self, joystick: Joystick):
        print(f"SECOND_ORDER: {ENABLE_2ND_ORDER}")
        angle_source = self.angle_source_selector.getSelected()
        self.ROBOT_MAX_TRANSLATIONAL = self.profile_selector.getSelected()[0]
        self.ROBOT_MAX_ROTATIONAL = self.profile_selector.getSelected()[1]
        self.MODULE_MAX_SPEED = self.profile_selector.getSelected()[0]

        # slew 
        # gives joystick ramping
        # linearX = self.slew_X.calculate(math.pow(joystick.getData()["axes"][1], 5)) * self.ROBOT_MAX_TRANSLATIONAL / self.move_scale_x
        # linearY = self.slew_Y.calculate(math.pow(joystick.getData()["axes"][0], 5)) * -self.ROBOT_MAX_TRANSLATIONAL / self.move_scale_y
        # angularZ = self.slew_Z.calculate(math.pow(joystick.getData()["axes"][3], 5)) * self.ROBOT_MAX_ROTATIONAL / self.turn_scale
        
        linearX = math.pow(joystick.getData()["axes"][1], 5) * self.ROBOT_MAX_TRANSLATIONAL / self.move_scale_x
        linearY = math.pow(joystick.getData()["axes"][0], 5) * -self.ROBOT_MAX_TRANSLATIONAL / self.move_scale_y
        angularZ = math.pow(joystick.getData()["axes"][3], 5) * self.ROBOT_MAX_ROTATIONAL / self.turn_scale

        self.linX = linearX
        self.linY = linearY
        self.angZ = angularZ

        if joystick.getData()["buttons"][7] == 1.0:
            self.field_oriented_value = not self.field_oriented_button.toggle(joystick.getData()["buttons"])
        else:
            self.field_oriented_value = not self.field_oriented_button.toggle(joystick.getData()["buttons"])

        if joystick.getData()["buttons"][6] == 1.0:
            self.navx.zeroYaw()
            if self.is_sim:
                self.navx_sim.zeroYaw()

        if joystick.getData()["axes"][5] == 1.0:
            self.slow = True
            self.move_scale_x = 1.0
            self.move_scale_y = 1.0
            self.turn_scale = 1.0
        else:            
            self.slow = False
            self.move_scale_x = 2.0
            self.move_scale_y = 2.0
            self.turn_scale = 2.0

        if joystick.getData()["axes"][6] == 1.0:
            self.auto_turn_value = "load"
        elif joystick.getData()["axes"][6] == -1.0:
            self.auto_turn_value = "score"
        else:
            self.auto_turn_value = "off"

        if self.field_oriented_value and self.auto_turn_value == "off":            
            # field  oriented
            navx_value = Rotation2d()
            if angle_source == "angle":
                navx_value = Rotation2d.fromDegrees(self.navx.getAngle()).__mul__(-1)
            elif angle_source == "yaw":
                navx_value = Rotation2d.fromDegrees(self.navx.getYaw()).__mul__(-1)
            elif angle_source == "normal":
                navx_value = self.navx.getRotation2d().__mul__(-1)
                
            if ENABLE_2ND_ORDER:
                self.speeds = self.correctForDynamics(ChassisSpeeds.fromFieldRelativeSpeeds(linearX, linearY, angularZ, navx_value))
            else:
                self.speeds = ChassisSpeeds.fromFieldRelativeSpeeds(linearX, linearY, angularZ, navx_value)
                
            print(f"X: {self.speeds.vx} Y: {self.speeds.vy} Z: {self.speeds.omega}")
        else:
            if ENABLE_2ND_ORDER:
                self.speeds = self.correctForDynamics(ChassisSpeeds(linearX, linearY, angularZ))
            else:    
                self.speeds = ChassisSpeeds(linearX, linearY, angularZ)
            #if self.last_print != f"linX: {round(self.speeds.vx, 2)} linY: {round(self.speeds.vy, 2)} angZ: {round(self.speeds.omega, 2)} MoveScaleX: {round(self.move_scale_x, 2)} MoveScaleY: {round(self.move_scale_y, 2)} TurnScale: {round(self.turn_scale, 2)}":
            #logging.info(f"linX: {round(self.speeds.vx, 2)} linY: {round(self.speeds.vy, 2)} angZ: {round(self.speeds.omega, 2)} MoveScaleX: {round(self.move_scale_x, 2)} MoveScaleY: {round(self.move_scale_y, 2)} TurnScale: {round(self.turn_scale, 2)}")
                #self.last_print = f"linX: {round(self.speeds.vx, 2)} linY: {round(self.speeds.vy, 2)} angZ: {round(self.speeds.omega, 2)} MoveScaleX: {round(self.move_scale_x, 2)} MoveScaleY: {round(self.move_scale_y, 2)} TurnScale: {round(self.turn_scale, 2)}"

        self.module_state = self.kinematics.toSwerveModuleStates(self.speeds)
        
        # normalize speeds
        # if the speeds are greater than the max speed, scale them down
        self.kinematics.desaturateWheelSpeeds(self.module_state, self.MODULE_MAX_SPEED)

        self.front_left_state: SwerveModuleState = self.module_state[0]
        self.front_right_state: SwerveModuleState = self.module_state[1]
        self.rear_left_state: SwerveModuleState = self.module_state[2]
        self.rear_right_state: SwerveModuleState = self.module_state[3]

        # optimize states
        #This makes the modules take the shortest path to the desired angle
        self.front_left_state = SwerveModuleState.optimize(self.front_left_state, Rotation2d(self.front_left.getEncoderPosition()))
        self.front_right_state = SwerveModuleState.optimize(self.front_right_state, Rotation2d(self.front_right.getEncoderPosition()))
        self.rear_left_state = SwerveModuleState.optimize(self.rear_left_state, Rotation2d(self.rear_left.getEncoderPosition()))
        self.rear_right_state = SwerveModuleState.optimize(self.rear_right_state, Rotation2d(self.rear_right.getEncoderPosition()))
        
        # using custom optimize
        # self.front_left_state = self.customOptimize(self.front_left_state, Rotation2d(self.front_left.getEncoderPosition()))
        # self.front_right_state = self.customOptimize(self.front_right_state, Rotation2d(self.front_right.getEncoderPosition()))
        # self.rear_left_state = self.customOptimize(self.rear_left_state, Rotation2d(self.rear_left.getEncoderPosition()))
        # self.rear_right_state = self.customOptimize(self.rear_right_state, Rotation2d(self.rear_right.getEncoderPosition()))

        self.front_left.set(self.front_left_state)
        self.front_right.set(self.front_right_state)
        self.rear_left.set(self.rear_left_state)
        self.rear_right.set(self.rear_right_state)
        
        

        #logging.info(f"angz: {angularZ}, FL: {self.front_left_state.speed}, FR: {self.front_right_state.speed}, BL: {self.rear_left_state.speed}, BR: {self.rear_right_state.speed}")

        self.motor_vels = [radiansToMeters(getWheelRadians(self.front_left.wheel_motor.getSelectedSensorVelocity(), "velocity")), radiansToMeters(getWheelRadians(self.front_right.wheel_motor.getSelectedSensorVelocity(), "velocity")), radiansToMeters(getWheelRadians(self.rear_left.wheel_motor.getSelectedSensorVelocity(), "velocity")), radiansToMeters(getWheelRadians(self.rear_right.wheel_motor.getSelectedSensorVelocity(), "velocity"))]
        self.motor_pos = [self.front_left.getEncoderPosition(), self.front_right.getEncoderPosition(), self.rear_left.getEncoderPosition(), self.rear_right.getEncoderPosition()]
        self.motor_temps = [self.front_left.wheel_motor.getTemperature(), self.front_right.wheel_motor.getTemperature() ,self.rear_left.wheel_motor.getTemperature(), self.rear_right.wheel_motor.getTemperature()]

        self.last_state = self.speeds
        
        if self.field_oriented_value:
            self.print = f"Navx: {self.navx.getRotation2d().__mul__(-1)} "
        else:
            self.print = ""

        #logging.info(f"FR: {self.front_left_state.speed}, {self.front_left_state.angle.radians()} | Current Angle: {self.front_left.getEncoderPosition()}")
        # logging.info(f"{self.print}linX: {round(self.speeds.vx, 2)} linY: {round(self.speeds.vy, 2)} angZ: {round(self.speeds.omega, 2)} FL: {round(radiansToMeters(getWheelRadians(self.front_left.wheel_motor.getSelectedSensorVelocity(), 'velocity')), 2)}")
        
    def getModuleCommand(self):
        data = dict()

        if self.is_sim:
            if ENABLE_2ND_ORDER:
                self.speeds = self.correctForDynamics(ChassisSpeeds.fromFieldRelativeSpeeds(self.linX, self.linY, self.angZ, self.navx_sim.getRotation2d().__mul__(-1)))
            else:
                self.speeds = ChassisSpeeds.fromFieldRelativeSpeeds(self.linX, self.linY, self.angZ, self.navx_sim.getRotation2d().__mul__(-1))
            #print(self.navx_sim.getRotation2d())
            
            self.module_state = self.kinematics.toSwerveModuleStates(self.speeds)
            
            # normalize speeds
            # if the speeds are greater than the max speed, scale them down
            self.kinematics.desaturateWheelSpeeds(self.module_state, self.speeds, self.MODULE_MAX_SPEED, self.ROBOT_MAX_TRANSLATIONAL, self.ROBOT_MAX_ROTATIONAL)
            
            if not self.locked:

                self.front_left_state: SwerveModuleState = self.module_state[0]
                self.front_right_state: SwerveModuleState = self.module_state[1]
                self.rear_left_state: SwerveModuleState = self.module_state[2]
                self.rear_right_state: SwerveModuleState = self.module_state[3]

                # optimize states
                #This makes the modules take the shortest path to the desired angle
                self.front_left_state = SwerveModuleState.optimize(self.front_left_state, Rotation2d(self.front_left.getEncoderPosition()))
                self.front_right_state = SwerveModuleState.optimize(self.front_right_state, Rotation2d(self.front_right.getEncoderPosition()))
                self.rear_left_state = SwerveModuleState.optimize(self.rear_left_state, Rotation2d(self.rear_left.getEncoderPosition()))
                self.rear_right_state = SwerveModuleState.optimize(self.rear_right_state, Rotation2d(self.rear_right.getEncoderPosition()))            
        
        m1_val = self.new_motion_magic_1.getNextVelocity(self.front_left_state.angle.radians(), self.front_left.getMotorPosition())
        m2_val = self.new_motion_magic_2.getNextVelocity(self.front_right_state.angle.radians(), self.front_right.getMotorPosition())
        m3_val = self.new_motion_magic_3.getNextVelocity(self.rear_left_state.angle.radians(), self.rear_left.getMotorPosition())
        m4_val = self.new_motion_magic_4.getNextVelocity(self.rear_right_state.angle.radians(), self.rear_right.getMotorPosition())
        data["name"] = getJointList()
        data["velocity"] = [
            metersToRadians(self.front_left_state.speed) * SCALING_FACTOR_FIX,
            m1_val * SCALING_FACTOR_FIX,
            metersToRadians(self.front_right_state.speed) * SCALING_FACTOR_FIX,
            m2_val * SCALING_FACTOR_FIX,
            metersToRadians(self.rear_left_state.speed) * SCALING_FACTOR_FIX,
            m3_val * SCALING_FACTOR_FIX,
            metersToRadians(self.rear_right_state.speed) * SCALING_FACTOR_FIX,
            m4_val * SCALING_FACTOR_FIX,
        ]
        data["position"] = [0.0]*8
        
        #print(f"{round(self.linX, 2)} {round(self.linY, 2)} {round(self.angZ, 2)} | {round(self.front_left.getMotorPosition(), 2)} {round(self.front_left.getMotorPosition(), 2)} {round(self.front_left.getMotorPosition(), 2)} {round(self.front_left.getMotorPosition(), 2)}")
        
        return data

    def swerveDriveAuton(self, linearX, linearY, angularZ):
        self.ROBOT_MAX_TRANSLATIONAL = self.profile_selector.getSelected()[0]
        self.ROBOT_MAX_ROTATIONAL = self.profile_selector.getSelected()[1]
        self.MODULE_MAX_SPEED = self.profile_selector.getSelected()[0]
        if ENABLE_2ND_ORDER:
            self.speeds = self.correctForDynamics(ChassisSpeeds(linearX*self.ROBOT_MAX_TRANSLATIONAL, -linearY*self.ROBOT_MAX_TRANSLATIONAL, angularZ*self.ROBOT_MAX_ROTATIONAL))
        else:
            self.speeds = ChassisSpeeds(linearX*self.ROBOT_MAX_TRANSLATIONAL, -linearY*self.ROBOT_MAX_TRANSLATIONAL, angularZ*self.ROBOT_MAX_ROTATIONAL)
        self.linX = linearX*self.ROBOT_MAX_TRANSLATIONAL
        self.linY = linearY*self.ROBOT_MAX_TRANSLATIONAL
        self.angZ = angularZ*self.ROBOT_MAX_ROTATIONAL

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
        
        # using custom optimize
        # self.front_left_state = self.customOptimize(self.front_left_state, Rotation2d(self.front_left.getEncoderPosition()))
        # self.front_right_state = self.customOptimize(self.front_right_state, Rotation2d(self.front_right.getEncoderPosition()))
        # self.rear_left_state = self.customOptimize(self.rear_left_state, Rotation2d(self.rear_left.getEncoderPosition()))
        # self.rear_right_state = self.customOptimize(self.rear_right_state, Rotation2d(self.rear_right.getEncoderPosition()))

        self.front_left.set(self.front_left_state)
        self.front_right.set(self.front_right_state)
        self.rear_left.set(self.rear_left_state)
        self.rear_right.set(self.rear_right_state)
        
    def swerveDriveAutonFieldOriented(self, linearX, linearY, angularZ):
        self.ROBOT_MAX_TRANSLATIONAL = self.profile_selector.getSelected()[0]
        self.ROBOT_MAX_ROTATIONAL = self.profile_selector.getSelected()[1]
        self.MODULE_MAX_SPEED = self.profile_selector.getSelected()[0]
        if ENABLE_2ND_ORDER:
            self.speeds = self.correctForDynamics(ChassisSpeeds.fromFieldRelativeSpeeds(linearX*self.ROBOT_MAX_TRANSLATIONAL, -linearY*self.ROBOT_MAX_TRANSLATIONAL, angularZ*self.ROBOT_MAX_ROTATIONAL, self.navx.getRotation2d().__mul__(-1)))
        else:
            self.speeds = ChassisSpeeds.fromFieldRelativeSpeeds(linearX*self.ROBOT_MAX_TRANSLATIONAL, -linearY*self.ROBOT_MAX_TRANSLATIONAL, angularZ*self.ROBOT_MAX_ROTATIONAL, self.navx.getRotation2d().__mul__(-1))
        self.linX = linearX*self.ROBOT_MAX_TRANSLATIONAL
        self.linY = linearY*self.ROBOT_MAX_TRANSLATIONAL
        self.angZ = angularZ*self.ROBOT_MAX_ROTATIONAL

        module_state = self.kinematics.toSwerveModuleStates(self.speeds)
        self.kinematics.desaturateWheelSpeeds(self.module_state, self.speeds, self.MODULE_MAX_SPEED, self.ROBOT_MAX_TRANSLATIONAL, self.ROBOT_MAX_ROTATIONAL)

        front_left_state: SwerveModuleState = module_state[0]
        front_right_state: SwerveModuleState = module_state[1]
        rear_left_state: SwerveModuleState = module_state[2]
        rear_right_state: SwerveModuleState = module_state[3]

        # optimize states
        front_left_state = SwerveModuleState.optimize(front_left_state, Rotation2d(self.front_left.getEncoderPosition()))
        front_right_state = SwerveModuleState.optimize(front_right_state, Rotation2d(self.front_right.getEncoderPosition()))
        rear_left_state = SwerveModuleState.optimize(rear_left_state, Rotation2d(self.rear_left.getEncoderPosition()))
        rear_right_state = SwerveModuleState.optimize(rear_right_state, Rotation2d(self.rear_right.getEncoderPosition()))
        
        # using custom optimize
        # self.front_left_state = self.customOptimize(self.front_left_state, Rotation2d(self.front_left.getEncoderPosition()))
        # self.front_right_state = self.customOptimize(self.front_right_state, Rotation2d(self.front_right.getEncoderPosition()))
        # self.rear_left_state = self.customOptimize(self.rear_left_state, Rotation2d(self.rear_left.getEncoderPosition()))
        # self.rear_right_state = self.customOptimize(self.rear_right_state, Rotation2d(self.rear_right.getEncoderPosition()))

        self.front_left.set(front_left_state)
        self.front_right.set(front_right_state)
        self.rear_left.set(rear_left_state)
        self.rear_right.set(rear_right_state)
        
    def lockDrive(self):
        self.locked = True
        self.front_left_state = SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        self.front_right_state = SwerveModuleState(0, Rotation2d.fromDegrees(45))
        self.rear_left_state = SwerveModuleState(0, Rotation2d.fromDegrees(45))
        self.rear_right_state = SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        
    def unlockDrive(self):
        self.locked = False
        self.front_left_state = SwerveModuleState(0, Rotation2d.fromDegrees(0))
        self.front_right_state = SwerveModuleState(0, Rotation2d.fromDegrees(0))
        self.rear_left_state = SwerveModuleState(0, Rotation2d.fromDegrees(0))
        self.rear_right_state = SwerveModuleState(0, Rotation2d.fromDegrees(0))
        





