from commands2 import *
from wpilib import Timer
from hardware_interface.joystick import Joystick
import wpimath
from wpimath import angleModulus
from wpimath.units import *
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians
from wpimath.controller import PIDController, ProfiledPIDController, ProfiledPIDControllerRadians
from hardware_interface.subsystems.drive_subsystem import DriveSubsystem
import logging
import math

class Units:
    METERS = 0
    FEET = 1
    INCHES = 2

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
        print(f"Time Elapsed: {self.timer.get()} s")
        print(f"Distance Traveled: {self.timer.get()*self.x} m")
        
    def end(self, interrupted):
        print("TIME END")
        self.drive.swerve_drive(0, 0, 0, False)
        self.drive.stop()
        
    def isFinished(self):
        return self.timer.get() >= self.seconds
        
class TurnToAngleCommand(CommandBase):
    def __init__(self, drive: DriveSubsystem, target):
        super().__init__()
        self.drive = drive
        self.constraints = TrapezoidProfileRadians.Constraints(5.0, 2.5)
        self.pid = ProfiledPIDControllerRadians(0.3, 0.0, 0.0, self.constraints)
        self.pid.enableContinuousInput(-math.pi, math.pi)
        self.target = math.radians(target)
        self.timer = Timer()
        self.addRequirements(self.drive)
        
    def clampToRange(self, value, min, max):
        if value > max:
            return max
        elif value < min:
            return min
        else:
            return value
        
    def initialize(self):
        self.timer.reset()
        self.timer.start()
        self.pid.setGoal(self.target)
        
    def execute(self):
        calc = self.pid.calculate(math.radians(self.drive.getGyroAngle180()))
        power = calc
        self.drive.swerve_drive(self.drive.getJoyVelocity().vx/self.drive.drivetrain.ROBOT_MAX_TRANSLATIONAL, self.drive.getJoyVelocity().vy/self.drive.drivetrain.ROBOT_MAX_TRANSLATIONAL, power, True)
        print(power/100.0)
        print("Pos: ", self.drive.getGyroAngle180())
        
    def end(self, interrupted):
        self.drive.swerve_drive(self.drive.getJoyVelocity().vx/self.drive.drivetrain.ROBOT_MAX_TRANSLATIONAL, self.drive.getJoyVelocity().vy/self.drive.drivetrain.ROBOT_MAX_TRANSLATIONAL, 0, False)
        self.drive.stop()
    
    def isFinished(self):
        return self.pid.atGoal()
        
class BalanceOnChargeStationCommand(CommandBase):
    def __init__(self, drive: DriveSubsystem, level_threshold: float):
        super().__init__()
        self.drive = drive
        self.power = 0.1
        self.level_threshold = level_threshold
        self.zero_count = 0
        self.addRequirements(self.drive)
        
    def initialize(self):
        self.drive.unlockDrive()
        
    def execute(self):
        print("Balance Pitch: ", self.drive.getGyroRoll180())
        if self.drive.getGyroRoll180() < 1:
            self.drive.swerve_drive(-self.power, 0, 0, False)
        elif self.drive.getGyroRoll180() > 1:
            self.drive.swerve_drive(self.power, 0, 0, False)
        if self.drive.getGyroRoll180() == 0:
            self.power /= 1.1
        
    def end(self, interrupted):
        self.drive.swerve_drive(0, 0, 0, False)
        print("LOCKING")
        self.drive.lockDrive()
        
    def isFinished(self):
        curr_angle = self.drive.getGyroRoll180()
        return abs(curr_angle) < self.level_threshold
    
# class BalanceOnChargePIDCommand(CommandBase):
#     def __init__(self, drive: DriveSubsystem, level_threshold: float):
#         super().__init__()
#         self.drive = drive
#         self.level_threshold = level_threshold
#         self.pid = PIDController(0.3, 0, 0.1)
#         # self.pid.enableContinuousInput(-180, 180)
#         self.pid.setTolerance(level_threshold)
        
#     def initialize(self):
#         self.pid.reset()
#         self.pid.setSetpoint(0)
        
#     def execute(self):
#         current_pitch = self.drive.getGyroRoll180()
#         pitch_power = self.pid.calculate(current_pitch)
#         print(-pitch_power)
#         self.drive.swerve_drive(-pitch_power/1000.0, 0, 0, False)
        
#     def end(self, interrupted):
#         self.drive.swerve_drive(0, 0, 0, False)
#         self.drive.lockDrive()
#         self.drive.stop()
    
#     def isFinished(self):
#         return self.pid.atSetpoint()
        
class DriveToChargeStationCommand(CommandBase):
    def __init__(self, drive: DriveSubsystem, tilt_threshold: float):
        super().__init__()
        self.drive = drive
        self.tilt_threshold = tilt_threshold
        self.addRequirements(self.drive)
        
    def initialize(self):
        self.drive.unlockDrive()
        
    def execute(self):
        print("To Charge Pitch: ", self.drive.getGyroRoll180())
        self.drive.swerve_drive(-3.5, 0, 0, False)
        
    def end(self, interrupted):
        self.drive.swerve_drive(0, 0, 0, False)
        self.drive.stop()
        
    def isFinished(self):
        return abs(self.drive.getGyroRoll180()) >= self.tilt_threshold
    
class TaxiAutoCommand(SequentialCommandGroup):
    def __init__(self, drive: DriveSubsystem, side: str):
        super().__init__()
        self.drive = drive
        self.side = side
        self.addRequirements(self.drive)
        if self.side == "bump":
            self.addCommands(
                PrintCommand("Starting Taxi Auto"),
                DriveTimeAutoCommand(self.drive, 1.75, (-1.5, 0, 0))
            )
        else:
            self.addCommands(
                PrintCommand("Starting Taxi Auto"),
                DriveTimeAutoCommand(self.drive, 1.0, (-1.5, 0, 0))
            )
        
class UnlockDriveCommand(CommandBase):
    def __init__(self, drive: DriveSubsystem):
        super().__init__()
        self.drive = drive
        self.addRequirements(self.drive)
        
    def initialize(self):
        self.drive.unlockDrive()
    
    def isFinished(self):
        return not self.drive.drivetrain.locked
    
class FieldOrientCommand(CommandBase):
    def __init__(self, drive: DriveSubsystem):
        super().__init__()
        self.drive = drive
        self.addRequirements(self.drive)
        
    def initialize(self):
        self.drive.unlockDrive()
        self.drive.hardResetGyro()
        self.drive.recalibrateGyro()
        
    
    