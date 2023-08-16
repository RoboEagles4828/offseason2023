from commands2 import SubsystemBase
from hardware_interface.drivetrain import DriveTrain

class DriveSubsystem(SubsystemBase):
    def __init__(self, drivetrain: DriveTrain):
        super().__init__()
        self.drivetrain = drivetrain
        
    def swerve_drive(self, x, y, z, field_oriented):
        if field_oriented:
            self.drivetrain.swerveDriveAutonFieldOriented(x/self.drivetrain.ROBOT_MAX_TRANSLATIONAL, y/self.drivetrain.ROBOT_MAX_TRANSLATIONAL, z/self.drivetrain.ROBOT_MAX_ROTATIONAL)
        else:
            self.drivetrain.swerveDriveAuton(x/self.drivetrain.ROBOT_MAX_TRANSLATIONAL, y/self.drivetrain.ROBOT_MAX_TRANSLATIONAL, z/self.drivetrain.ROBOT_MAX_ROTATIONAL)
    
    def resetGyro(self):
        self.drivetrain.navx.zeroYaw()
        
    def getEncoderData(self):
        return self.drivetrain.getEncoderData()
    
    def getGyroAngle180(self):
        return self.drivetrain.navx.getYaw()
        
    def stop(self):
        self.drivetrain.stop()
        