from commands2 import *
from wpilib import Timer
import wpimath
from wpimath.controller import PIDController
from hardware_interface.subsystems.drive_subsystem import DriveSubsystem
import logging

# Drive time command

class DriveTimeAutoCommand(CommandBase):
    def __init__(self, drive: DriveSubsystem, seconds: float, velocity: tuple[float, float, float]):
        super().__init__()
        self.drive = drive
        self.seconds = seconds
        self.x = velocity[0]
        self.y = velocity[1]
        self.z = velocity[2]
        self.timer = Timer()
        self.addRequirements(self.drive)
        
    def initialize(self):
        self.timer.reset()
        self.timer.start()        

    def execute(self):
        self.drive.swerve_drive(self.x, self.y, self.z, True)
        # print(f"DriveTimeAuton Runtime: {self.timer.get()}")
        
    def end(self, interrupted):
        self.drive.swerve_drive(0, 0, 0, True)
        self.drive.stop()
        
    def isFinished(self):
        return self.timer.hasElapsed(self.seconds)
    
class TurnToAngleCommand(CommandBase):
    def __init__(self, drive: DriveSubsystem, angle: float, relative: bool):
        super().__init__()
        self.drive = drive
        self.angle = angle
        self.target = 0
        self.relative = relative
        self.turn_pid = PIDController(0, 0, 0)
        self.turn_pid.enableContinuousInput(-180, 180)
        self.addRequirements(self.drive)
        
    def initialize(self):
        self.turn_pid.calculate()
        current_angle = self.drive.getGyroAngle180()
        if self.relative:
            self.target = current_angle + self.angle
        else:
            self.target = self.angle
        
    def execute(self):
        current_angle = self.drive.getGyroAngle180()
        self.drive.swerve_drive(0, 0, self.turn_pid.calculate(current_angle, self.target), True)
        
    def end(self, interrupted):
        self.drive.swerve_drive(0, 0, 0, True)
        self.drive.stop()
        
    def isFinished(self):
        return self.turn_pid.atSetpoint()
    
class TaxiAutoCommand(SequentialCommandGroup):
    def __init__(self, drive: DriveSubsystem):
        super().__init__()
        self.drive = drive
        self.addRequirements(self.drive)
        self.addCommands(
            WaitCommand(0.5),
            DriveTimeAutoCommand(self.drive, 3.0, (-3.5, 0, 0))
        )
        
    
    