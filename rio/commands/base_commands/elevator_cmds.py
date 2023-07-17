from commands2 import CommandBase
from hardware_interface.armcontroller import ArmController

class ElevatorHighCommand(CommandBase):
    def __init__(self, arm_controller: ArmController):
        CommandBase.__init__(self)
        self.arm_controller = arm_controller

    def initialize(self):
        self.arm_controller.elevator_high_level_on()

    def isFinished(self):
        return True

class ElevatorMidCommand(CommandBase):
    def __init__(self, arm_controller: ArmController):
        CommandBase.__init__(self)
        self.arm_controller = arm_controller

    def initialize(self):
        self.arm_controller.elevator_mid_level_on()

    def isFinished(self):
        return True

class ElevatorRetractCommand(CommandBase):
    def __init__(self, arm_controller: ArmController):
        CommandBase.__init__(self)
        self.arm_controller = arm_controller

    def initialize(self):
        self.arm_controller.elevator.setPosition(self.arm_controller.elevator.min)

    def isFinished(self):
        return True

    
