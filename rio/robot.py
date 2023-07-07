from hardware_interface.drivetrain import DriveTrain
from hardware_interface.joystick import Joystick
from hardware_interface.armcontroller import ArmController
import wpilib
from wpilib.shuffleboard import Shuffleboard
from auton_selector import AutonSelector

class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.arm_controller = ArmController()
        self.drive_train = DriveTrain()
        self.joystick = Joystick()
        self.auton_selector = AutonSelector(self.arm_controller, self.drive_train)
        self.auton_run = False
        self.shuffleboard = Shuffleboard.getTab("Main")
        self.shuffleboard.add(self.auton_selector.autonChooser)


    def teleopPeriodic(self):
        self.drive_train.swerveDrive(self.joystick)
        self.arm_controller.setArm(self.joystick)

if __name__ == '__main__':
    wpilib.run(Robot)