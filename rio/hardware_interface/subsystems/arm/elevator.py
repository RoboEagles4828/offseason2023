import wpilib
import wpilib.simulation
import ctre
import ctre.sensors
import time
import logging
from hardware_interface.subsystems.arm.arm_constants import *

class Elevator():
    def __init__(self, port : int, min : float = 0.0, max : float = 1.0):
        self.motor = ctre.WPI_TalonFX(port, "rio")
        commonTalonSetup(self.motor)
        self.min = min
        self.max = max
        self.totalTicks = TICKS_PER_REVOLUTION * TOTAL_ELEVATOR_REVOLUTIONS
        self.lastCommand = None
        # Phase
        self.motor.setSensorPhase(False)
        self.motor.setInverted(False)
        # Frames
        self.motor.setStatusFramePeriod(ctre.StatusFrameEnhanced.Status_10_MotionMagic, 10, MOTOR_TIMEOUT)
        # Motion Magic
        self.motor.configMotionCruiseVelocity(MOTOR_PID_CONFIG['MAX_SPEED'], MOTOR_TIMEOUT) # Sets the maximum speed of motion magic (ticks/100ms)
        self.motor.configMotionAcceleration(MOTOR_PID_CONFIG['TARGET_ACCELERATION'], MOTOR_TIMEOUT) # Sets the maximum acceleration of motion magic (ticks/100ms)
        # self.motor.configClearPositionOnLimitR(True)

    def getPosition(self) -> float:
        percent = self.motor.getSelectedSensorPosition() / self.totalTicks
        return percent * (self.max - self.min) + self.min
    
    def getVelocity(self) -> float:
        percent = (self.motor.getSelectedSensorVelocity() * 10) / self.totalTicks
        return percent * (self.max - self.min) + self.min

    def stop(self):
        self.motor.set(ctre.TalonFXControlMode.PercentOutput, 0)

    def setPosition(self, position : float):
        if position != self.lastCommand:
            logging.info(f"Elevator Position: {position}")
            percent = (position - self.min) / (self.max - self.min)
            self.motor.set(ctre.TalonFXControlMode.MotionMagic, percent * self.totalTicks)
            self.lastCommand = position
        else:
            logging.info(f"Elevator Position: {position}")