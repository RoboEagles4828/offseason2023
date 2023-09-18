import wpilib
import wpilib.simulation
import ctre
import ctre.sensors
import time
import logging
import math
from hardware_interface.subsystems.arm.arm_constants import *

class Piston():
    def __init__(self, hub : wpilib.PneumaticHub, ports : "list[int]", min : float = 0.0, max : float = 1.0, reverse : bool = False, name : str = "Piston"):
        self.solenoid = hub.makeDoubleSolenoid(ports[0], ports[1])
        self.state = int(self.solenoid.get() != wpilib.DoubleSolenoid.Value.kForward)
        self.min = min
        self.max = max
        self.reverse = reverse
        self.name = name
        self.lastCommand = None

    def getPosition(self):
        # return float(self.state) * (self.max - self.min) + self.min
        ret = 0.0
        if self.lastCommand:
            ret = self.lastCommand
        return ret
    
    # The Solenoids don't have a velocity value, so we set it to zero here
    def getVelocity(self):
        return 0.0
    
    def stop(self):
        self.solenoid.set(wpilib.DoubleSolenoid.Value.kOff)

    def setPosition(self, position : float):
        if position != self.lastCommand:
            logging.info(f"{self.name} Position: {position}")
            center = abs((self.max - self.min) / 2 + self.min)
            forward = wpilib.DoubleSolenoid.Value.kForward
            reverse = wpilib.DoubleSolenoid.Value.kReverse
            if abs(position) >= center:
                logging.info(f"{self.name} first block")
                self.solenoid.set(forward)
            elif abs(position) < center:
                logging.info(f"{self.name} second block")
                self.solenoid.set(reverse)
            self.lastCommand = position
        else:
            logging.info(f"{self.name} Position: {position}")