import wpilib
from wpimath.geometry._geometry import *
from wpimath.kinematics import SwerveModuleState
from hardware_interface.subsystems.swerve.swerve_constants import *
import ctre
import ctre.sensors
import math

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
        supply_current_limit = 30
        supply_current_threshold = 40
        supply_current_threshold_time = 0.1
        supply_current_limit_configs = ctre.SupplyCurrentLimitConfiguration(True, supply_current_limit, supply_current_threshold, supply_current_threshold_time)
        self.wheel_motor.configSupplyCurrentLimit(supply_current_limit_configs, timeout_ms)
        
        # Stator Current Limit
        stator_current_limit = 40 # TerrorBytes/Yeti: 80
        stator_current_threshold = 80 # TerrorBytes/Yeti: 120
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
        supply_current_limit_configs = ctre.SupplyCurrentLimitConfiguration(False, supply_current_limit, supply_current_threshold, supply_current_threshold_time)
        self.axle_motor.configSupplyCurrentLimit(supply_current_limit_configs, timeout_ms)

        # Stator Current Limit
        stator_current_limit = 40
        stator_current_threshold = 80
        stator_current_threshold_time = 0.1
        stator_current_limit_configs = ctre.StatorCurrentLimitConfiguration(False, stator_current_limit, stator_current_threshold, stator_current_threshold_time)
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