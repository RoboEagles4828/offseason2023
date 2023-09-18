import wpilib
import wpilib.simulation
import ctre
import ctre.sensors
from hardware_interface.subsystems.arm.arm_constants import *

NAMESPACE = 'real'
CMD_TIMEOUT_SECONDS = 1
MOTOR_TIMEOUT = 30 # 0 means do not use the timeout
TICKS_PER_REVOLUTION = 2048.0
TOTAL_ELEVATOR_REVOLUTIONS = 164
TOTAL_INTAKE_REVOLUTIONS = 6
SCALING_FACTOR_FIX = 10000

# Port Numbers for all of the Solenoids and other connected things
# The numbers below will **need** to be changed to fit the robot wiring
PORTS = {
    # Modules
    'HUB': 18,
    # Pistons
    'ARM_ROLLER_BAR': [14, 15],
    'TOP_GRIPPER_SLIDER': [10, 11],
    'TOP_GRIPPER': [12, 13],
    # Wheels
    'ELEVATOR': 13,
    # 'BOTTOM_GRIPPER_LIFT': 14
}

MOTOR_PID_CONFIG = {
    'SLOT': 2,
    'MAX_SPEED': 32000*1.5,             # Ticks/100ms 
    'TARGET_ACCELERATION': 30000*1.5,    # Ticks/100ms
    "kP": 0.2,
    "kI": 0.0,
    "kD": 0.1,
    "kF": 0.2,
}

JOINT_LIST = [
    'arm_roller_bar_joint',
    'top_slider_joint',
    'top_gripper_left_arm_joint',
    'elevator_center_joint',
]

def getJointList():
    return JOINT_LIST

def commonTalonSetup(talon : ctre.WPI_TalonFX):
    talon.configFactoryDefault(MOTOR_TIMEOUT)
    talon.configNeutralDeadband(0.01, MOTOR_TIMEOUT)

    # Voltage
    talon.configVoltageCompSaturation(12, MOTOR_TIMEOUT)
    talon.enableVoltageCompensation(True)
    
    # Sensor
    talon.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor, 0, MOTOR_TIMEOUT)
    talon.configIntegratedSensorInitializationStrategy(ctre.sensors.SensorInitializationStrategy.BootToZero)

    # PID
    talon.selectProfileSlot(MOTOR_PID_CONFIG['SLOT'], 0)
    talon.config_kP(MOTOR_PID_CONFIG['SLOT'], MOTOR_PID_CONFIG['kP'], MOTOR_TIMEOUT)
    talon.config_kI(MOTOR_PID_CONFIG['SLOT'], MOTOR_PID_CONFIG['kI'], MOTOR_TIMEOUT)
    talon.config_kD(MOTOR_PID_CONFIG['SLOT'], MOTOR_PID_CONFIG['kD'], MOTOR_TIMEOUT)
    talon.config_kD(MOTOR_PID_CONFIG['SLOT'], MOTOR_PID_CONFIG['kF'], MOTOR_TIMEOUT)
    
    # Nominal and Peak
    talon.configNominalOutputForward(0, MOTOR_TIMEOUT)
    talon.configNominalOutputReverse(0, MOTOR_TIMEOUT)
    talon.configPeakOutputForward(1, MOTOR_TIMEOUT)
    talon.configPeakOutputReverse(-1, MOTOR_TIMEOUT)
    return