from commands2 import SubsystemBase
from wpimath.kinematics import ChassisSpeeds
from hardware_interface.drivetrain import DriveTrain
from hardware_interface.joystick import Joystick
import math

class DriveSubsystem(SubsystemBase):
    def __init__(self, drivetrain: DriveTrain, joystick: Joystick):
        super().__init__()
        self.drivetrain = drivetrain
        self.joystick = joystick
        
    def swerve_drive(self, x, y, z, field_oriented):
        if field_oriented:
            self.drivetrain.swerveDriveAutonFieldOriented(x, y, z)
        else:
            self.drivetrain.swerveDriveAuton(x, y, z)
            
    def getWheelEncoderPositions(self):
        return [
            self.drivetrain.front_left.getEncoderData()[0]["position"],
            self.drivetrain.front_right.getEncoderData()[0]["position"],
            self.drivetrain.rear_left.getEncoderData()[0]["position"],
            self.drivetrain.rear_right.getEncoderData()[0]["position"]
        ]
        
    def getWheelEncoderVelocities(self):
        return [
            self.drivetrain.front_left.getEncoderData()[0]["velocity"],
            self.drivetrain.front_right.getEncoderData()[0]["velocity"],
            self.drivetrain.rear_left.getEncoderData()[0]["velocity"],
            self.drivetrain.rear_right.getEncoderData()[0]["velocity"]
        ]
        
    def metersToShaftTicks(self, meters):
        return self.drivetrain.metersToShaftTicks(meters)
    
    def shaftTicksToMeters(self, ticks):
        return self.drivetrain.shaftTicksToMeters(ticks)
            
    def getKinematics(self):
        return self.drivetrain.kinematics
    
    def resetGyro(self):
        if self.drivetrain.is_sim:
            self.drivetrain.navx_sim.zeroYaw()
        self.drivetrain.navx.zeroYaw()
        
    def hardResetGyro(self):
        if self.drivetrain.is_sim:
            self.drivetrain.navx_sim.zeroYaw()
        self.drivetrain.navx.reset()
        
    def recalibrateGyro(self):
        self.drivetrain.navx.calibrate()
        
    def getEncoderData(self):
        return self.drivetrain.getEncoderData()
    
    def getGyroAngle180(self):
        if self.drivetrain.is_sim:
            return self.drivetrain.navx_sim.getYawDegrees()
        return self.drivetrain.navx.getYaw()
    
    def getGyroRoll180(self):
        if self.drivetrain.is_sim:
            return self.drivetrain.navx_sim.getRollDegrees()
        return self.drivetrain.navx.getRoll()
    
    def getGyroPitch180(self):
        if self.drivetrain.is_sim:
            return self.drivetrain.navx_sim.getPitchDegrees()
        return self.drivetrain.navx.getPitch()
    
    def getVelocity(self):
        return self.drivetrain.speeds
    
    def getJoyVelocity(self):
        linearX = math.pow(self.joystick.getData()["axes"][1], 5) * self.drivetrain.ROBOT_MAX_TRANSLATIONAL / self.drivetrain.move_scale_x
        linearY = math.pow(self.joystick.getData()["axes"][0], 5) * -self.drivetrain.ROBOT_MAX_TRANSLATIONAL / self.drivetrain.move_scale_y
        return ChassisSpeeds(linearX, linearY, 0)
    
    def lockDrive(self):
        self.drivetrain.lockDrive()
        
    def unlockDrive(self):
        self.drivetrain.unlockDrive()
        
    def percentToMeters(self, percent):
        return percent * 5.0
        
    def stop(self):
        self.drivetrain.stop()
        