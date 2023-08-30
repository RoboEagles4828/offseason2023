from commands2 import *
from wpilib import Timer
import wpimath
from wpimath.controller import PIDController
from hardware_interface.subsystems.drive_subsystem import DriveSubsystem
import logging
import math

class MainDriveCommand(CommandBase):
    def __init__(self, drive: DriveSubsystem, joystick):
        super().__init__()
        self.drive = drive
        self.joystick = joystick
        self.addRequirements(self.drive)
    
    def initialize(self):
        self.drive.unlockDrive()
        
    def execute(self):
        self.drive.swerve_drive(self.joystick)
    
    def end(self, interrupted):
        self.drive.stop()
        
    def isFinished(self):
        return False

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
    def __init__(self, drive: DriveSubsystem, angle: float, relative: bool, other_velocities=(0, 0)):
        super().__init__()
        self.drive = drive
        self.angle = angle
        self.target = 0
        self.relative = relative
        self.turn_pid = PIDController(0.3, 0, 0.1)
        self.turn_pid.enableContinuousInput(-180, 180)
        self.turn_pid.setTolerance(5)
        self.other = [i/self.drive.drivetrain.ROBOT_MAX_TRANSLATIONAL for i in other_velocities]
        self.addRequirements(self.drive)
        
    def initialize(self):
        logging.info("TurnToAngleCommand initialized")
        current_angle = self.drive.getGyroAngle180()
        if self.relative:
            self.target = current_angle + self.angle
        else:
            self.target = self.angle
            
    def clampToRange(self, value, min, max):
        if value > max:
            return max
        elif value < min:
            return min
        else:
            return value
        
    def execute(self):
        current_angle = self.drive.getGyroAngle180()
        turn_power = self.turn_pid.calculate(current_angle, self.target)
        logging.info(f"TurnToAngleCommand executing, target: {self.target} current: {self.drive.getGyroAngle180()} power: {turn_power/1000.0}")
        self.drive.swerve_drive(self.other[0], self.other[1], turn_power/1000.0, True)
        
    def end(self, interrupted):
        logging.info("TurnToAngleCommand ended")
        self.drive.swerve_drive(self.other[0], self.other[1], 0, True)
        self.drive.stop()
        
    def isFinished(self):
        return self.turn_pid.atSetpoint()
    
class TurnToAngleAutonCommand(CommandBase):
    def __init__(self, drive: DriveSubsystem, angle: float, relative: bool, other_velocities=(0, 0)):
        super().__init__()
        self.drive = drive
        self.angle = angle
        self.target = 0
        self.relative = relative
        self.turn_pid = PIDController(0.2, 0, 0.1)
        self.turn_pid.enableContinuousInput(-180, 180)
        self.turn_pid.setTolerance(5)
        self.other = [i/self.drive.drivetrain.ROBOT_MAX_TRANSLATIONAL for i in other_velocities]
        self.addRequirements(self.drive)
        
    def initialize(self):
        logging.info("TurnToAngleCommand initialized")
        current_angle = self.drive.getGyroAngle180()
        if self.relative:
            self.target = current_angle + self.angle
        else:
            self.target = self.angle
            
    def clampToRange(self, value, min, max):
        if value > max:
            return max
        elif value < min:
            return min
        else:
            return value
        
    def execute(self):
        current_angle = self.drive.getGyroAngle180()
        turn_power = self.turn_pid.calculate(current_angle, self.target)
        logging.info(f"TurnToAngleCommand executing, target: {self.target} current: {self.drive.getGyroAngle180()} power: {turn_power/100.0}")
        self.drive.swerve_drive(self.other[0], self.other[1], turn_power/100.0, True)
        
    def end(self, interrupted):
        logging.info("TurnToAngleCommand ended")
        self.drive.swerve_drive(self.other[0], self.other[1], 0, True)
        self.drive.stop()
        
    def isFinished(self):
        return self.turn_pid.atSetpoint()
    
class BalanceOnChargeStationCommand(CommandBase):
    def __init__(self, drive: DriveSubsystem, level_threshold: float):
        super().__init__()
        self.drive = drive
        self.level_threshold = level_threshold
        self.pitch_controller = PIDController(0.3, 0, 0.1)
        self.pitch_controller.enableContinuousInput(-180, 180)
        self.pitch_controller.setTolerance(5)
        self.addRequirements(self.drive)
        
    def initialize(self):
        self.pitch_controller.reset()
        self.pitch_controller.setSetpoint(self.level_threshold)
        
    def execute(self):
        current_pitch = self.drive.getGyroPitch180()
        pitch_power = self.pitch_controller.calculate(current_pitch)
        self.drive.swerve_drive(-pitch_power/1000.0, 0, 0, True)
        
    def end(self, interrupted):
        self.drive.swerve_drive(0, 0, 0, True)
        self.drive.stop()
        self.drive.lockDrive()
        
    def isFinished(self):
        return self.pitch_controller.atSetpoint()
        
class DriveToChargeStationCommand(CommandBase):
    def __init__(self, drive: DriveSubsystem, tilt_threshold: float):
        super().__init__()
        self.drive = drive
        self.tilt_threshold = tilt_threshold
        self.addRequirements(self.drive)
        
    def initialize(self):
        self.drive.unlockDrive()
        
    def execute(self):
        self.drive.swerve_drive(-0.5, 0, 0, True)
        
    def end(self, interrupted):
        self.drive.swerve_drive(0, 0, 0, True)
        self.drive.stop()
        
    def isFinished(self):
        return self.drive.getGyroPitch180() >= self.tilt_threshold
class TaxiAutoCommand(SequentialCommandGroup):
    def __init__(self, drive: DriveSubsystem):
        super().__init__()
        self.drive = drive
        self.addRequirements(self.drive)
        self.addCommands(
            WaitCommand(0.5),
            DriveTimeAutoCommand(self.drive, 1.5, (-3.5, 0, 0))
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
        self.drive.resetGyro()
        
        
class PostAutonCommand(SequentialCommandGroup):
    def __init__(self, drive: DriveSubsystem):
        super().__init__()
        self.drive = drive
        self.addRequirements(self.drive)
        self.addCommands(
            UnlockDriveCommand(self.drive),
            TurnToAngleAutonCommand(self.drive, 180, False),
            FieldOrientCommand(self.drive)
        )
        
    
    