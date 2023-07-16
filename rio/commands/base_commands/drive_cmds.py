import typing
from commands2 import CommandBase
from commands2._impl import ParallelRaceGroup
from hardware_interface.drivetrain import DriveTrain
from wpimath.controller import PIDController

class MoveCommand(CommandBase):
    def __init__(self, drive_train: DriveTrain, distance, linearX, linearY, angularZ):
        self.drive_train = drive_train
        self.distance = distance
        self.linearX = linearX
        self.linearY = linearY
        self.angularZ = angularZ

        #                     P  I  D
        self.pid_constants = [0, 0, 0]
        self.movement_pid = PIDController(*self.pid_constants)
        self.movement_pid.setSetpoint(self.inches)
        self.movement_pid.calculate()

    def execute(self):
        self.drive_train.swerveDriveAuton(self.linearX, self.linearY, self.angularZ)
    
    def calculateDistanceTraveled(self) -> int:
        # caluculate distance traveled by robot from encoders
        # if distance traveled is greater than distance, return 1
        # if distance traveled is less than distance, return -1
        # if distance traveled is equal to distance, return 0

        wheel_motor_encoder_positions = [
            self.drive_train.front_left.getEncoderData()[0]["position"]/10000.0,
            self.drive_train.front_right.getEncoderData()[0]["position"]/10000.0,
            self.drive_train.back_left.getEncoderData()[0]["position"]/10000.0,
            self.drive_train.back_right.getEncoderData()[0]["position"]/10000.0
        ]


    