import typing
from commands2 import CommandBase
from commands2._impl import ParallelRaceGroup
from hardware_interface.drivetrain import DriveTrain
from wpimath.controller import PIDController
import math

class MoveCommand(CommandBase):
    def __init__(self, drive_train: DriveTrain, meters, linearX, linearY, angularZ):
        self.drive_train = drive_train
        self.meters = meters
        self.linearX = linearX
        self.linearY = linearY
        self.angularZ = angularZ

        self.wheel_radius = 0.0508
        self.wheel_circumference = 2 * math.pi * self.wheel_radius

    def initialize(self):
        self.encoder_targets = [
            self.drive_train.front_left.getEncoderData()[0]["position"]/10000.0 + self.meters/self.wheel_circumference,
            self.drive_train.front_right.getEncoderData()[0]["position"]/10000.0 + self.meters/self.wheel_circumference,
            self.drive_train.rear_left.getEncoderData()[0]["position"]/10000.0 + self.meters/self.wheel_circumference,
            self.drive_train.rear_right.getEncoderData()[0]["position"]/10000.0 + self.meters/self.wheel_circumference
        ]


    def execute(self):
        self.drive_train.swerveDriveAuton(self.linearX, self.linearY, self.angularZ)

    def end(self):
        self.drive_train.swerveDriveAuton(0, 0, 0)
    
    def isFinished(self):
        front_left_radians = self.drive_train.front_left.getEncoderData()[0]["position"]/10000.0
        front_right_radians = self.drive_train.front_right.getEncoderData()[0]["position"]/10000.0
        rear_left_radians = self.drive_train.rear_left.getEncoderData()[0]["position"]/10000.0
        rear_right_radians = self.drive_train.rear_right.getEncoderData()[0]["position"]/10000.0

        if (front_left_radians >= self.encoder_targets[0] and front_right_radians >= self.encoder_targets[1] and rear_left_radians >= self.encoder_targets[2] and rear_right_radians >= self.encoder_targets[3]):
            return True
        else:
            return False

        


    