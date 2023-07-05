from hardware_interface.drivetrain import DriveTrain
from hardware_interface.joystick import Joystick
from hardware_interface.armcontroller import ArmController
import wpilib

class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.arm_controller = ArmController()
        self.drive_train = DriveTrain()
        self.joystick = Joystick()

    def teleopPeriodic(self):
        self.drive_train.swerveDrive(self.joystick)
        self.arm_controller.setArm(self.joystick)

if __name__ == '__main__':
    wpilib.run(Robot)