from commands2 import *
from wpilib import Timer
import wpimath
from wpimath.controller import PIDController
from hardware_interface.subsystems.arm_subsystem import ArmSubsystem, ElevatorState, PivotState, GrabberState
from hardware_interface.commands.do_nothing_command import DoNothingCommand
import logging

class ElevatorPivotCommand(CommandBase):
    def __init__(self, arm: ArmSubsystem, state: PivotState):
        super().__init__()
        self.arm = arm
        self.state = state
        self.timer = Timer()
        self.addRequirements(self.arm)
        
    def initialize(self):
        self.timer.reset()
        self.timer.start()        

    def execute(self):
        self.arm.setPivot(self.state)
        
    def end(self, interrupted):
        self.arm.stop()
        
    def isFinished(self):
        return self.timer.hasElapsed(0.5)
    
class ElevatorPresetCommand(CommandBase):
    def __init__(self, arm: ArmSubsystem, state: ElevatorState):
        super().__init__()
        self.arm = arm
        self.state = state
        self.timer = Timer()
        self.addRequirements(self.arm)
        
    def initialize(self):
        self.timer.reset()
        self.timer.start()
    
    def execute(self):
        self.arm.setElevator(self.state)
        
    def end(self, interrupted):
        self.arm.stop()
        
    def isFinished(self):
        if self.state == ElevatorState.HOME:
            return self.timer.hasElapsed(3)
        elif self.state == ElevatorState.MID:
            return self.timer.hasElapsed(2)
        elif self.state == ElevatorState.HIGH:
            return self.timer.hasElapsed(4)
        elif self.state == ElevatorState.LOADING_STATION:
            return self.timer.hasElapsed(1)
        
class TopGripperCommand(CommandBase):
    def __init__(self, arm: ArmSubsystem, state: GrabberState):
        super().__init__()
        self.arm = arm
        self.state = state
        self.timer = Timer()
        self.addRequirements(self.arm)
        
    def initialize(self):
        self.timer.reset()
        self.timer.start()
    
    def execute(self):
        self.arm.setGrabber(self.state)
        
    def end(self, interrupted):
        self.arm.stop()
        
    def isFinished(self):
        return self.timer.hasElapsed(0.5)
        
class ScoreCommand(SequentialCommandGroup):
    def __init__(self, arm: ArmSubsystem, state: ElevatorState, gamepiece: str):
        super().__init__()
        self.addCommands(
            ElevatorPresetCommand(arm, state),
            TopGripperCommand(arm, GrabberState.OPEN),
            WaitCommand(0.5),
            ElevatorPresetCommand(arm, ElevatorState.HOME),
        )
        
        