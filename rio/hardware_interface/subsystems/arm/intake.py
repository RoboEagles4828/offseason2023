import wpilib
import wpilib.simulation
import ctre
import ctre.sensors
import time
import logging
import math
from hardware_interface.subsystems.arm.arm_constants import *

class Intake():
    def __init__(self, port : int, min : float = 0.0, max : float = 1.0):
        self.motor = ctre.WPI_TalonFX(port, "rio")
        commonTalonSetup(self.motor)
        self.state = 0
        self.min = min
        self.max = max
        self.totalTicks = TICKS_PER_REVOLUTION * TOTAL_INTAKE_REVOLUTIONS
        self.lastCommand = None
        # Phase
        self.motor.setSensorPhase(False)
        self.motor.setInverted(False)
        # Frames
        self.motor.setStatusFramePeriod(ctre.StatusFrameEnhanced.Status_13_Base_PIDF0, 10, MOTOR_TIMEOUT)
        # Brake
        self.motor.setNeutralMode(ctre.NeutralMode.Brake)

    def getPosition(self):
        percent = self.motor.getSelectedSensorPosition() / self.totalTicks
        return percent * (self.max - self.min) + self.min

    def getVelocity(self):
        percent = self.motor.getSelectedSensorVelocity() * 10 / self.totalTicks
        return percent * (self.max - self.min) + self.min

    def stop(self):
        self.motor.set(ctre.TalonFXControlMode.PercentOutput, 0)

    def setPosition(self, position : float):
        logging.info(f"Intake Position: {position}")
        
        # Moves the intake
        center = (self.max - self.min) / 2 + self.min
        if position >= center and self.state == 0:
            self.motor.set(ctre.TalonFXControlMode.Velocity, -TICKS_PER_REVOLUTION/2)
            self.state = 1
        elif position < center and self.state == 1:
            self.motor.set(ctre.TalonFXControlMode.Velocity, TICKS_PER_REVOLUTION/2)
            self.state = 0
        
        if self.state == 0 and not self.motor.isFwdLimitSwitchClosed():
            self.motor.set(ctre.TalonFXControlMode.Velocity, TICKS_PER_REVOLUTION/2)
        elif self.state == 1 and not self.motor.isRevLimitSwitchClosed():
            self.motor.set(ctre.TalonFXControlMode.Velocity, -TICKS_PER_REVOLUTION/2)