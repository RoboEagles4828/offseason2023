from commands2 import CommandBase
from hardware_interface.armcontroller import ArmController

class GrabCommand(CommandBase):
    def __init__(self, arm_controller: ArmController):
        self.arm_controller = arm_controller

    def initialize(self):
        self.arm_controller.top_gripper_control_on()

    def isFinished(self):
        return True
    
class ReleaseCommand(CommandBase):
    def __init__(self, arm_controller: ArmController):
        self.arm_controller = arm_controller

    def initialize(self):
        self.arm_controller.top_gripper_control_off()

    def isFinished(self):
        return True
    
class PivotUpCommand(CommandBase):
    def __init__(self, arm_controller: ArmController):
        self.arm_controller = arm_controller

    def initialize(self):
        self.arm_controller.elevator_pivot_control_on()

    def isFinished(self):
        return True
    
class PivotDownCommand(CommandBase):
    def __init__(self, arm_controller: ArmController):
        self.arm_controller = arm_controller

    def initialize(self):
        self.arm_controller.elevator_pivot_control_off()

    def isFinished(self):
        return True