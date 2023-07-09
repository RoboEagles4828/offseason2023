from hardware_interface.drivetrain import DriveTrain
from hardware_interface.joystick import Joystick
from hardware_interface.armcontroller import ArmController
import wpilib
from wpilib.shuffleboard import Shuffleboard
from wpilib.shuffleboard import SuppliedFloatValueWidget
from auton_selector import AutonSelector
import time

class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.arm_controller = ArmController()
        self.drive_train = DriveTrain()
        self.joystick = Joystick("ps4")
        self.auton_selector = AutonSelector(self.arm_controller, self.drive_train)
        self.auton_run = False

        self.shuffleboard = Shuffleboard.getTab("Main")
        self.shuffleboard.add(title="AUTON", defaultValue=self.auton_selector.autonChooser)

        self.shuffleboard.add("PROFILE", self.drive_train.profile_selector)
        self.shuffleboard.addDouble("YAW", lambda: (self.drive_train.navx.getRotation2d().degrees()))
        self.shuffleboard.addBoolean("FIELD ORIENTED", lambda: (self.drive_train.field_oriented_value))

    def autonomousInit(self):
        self.auton_selector.timer_reset()
        self.auton_selector.set_start_time(self.auton_selector.timer.getFPGATimestamp())
        self.arm_controller.top_gripper_control_on()

    def autonomousPeriodic(self):
        self.auton_selector.run()

    def teleopInit(self):
        self.drive_train.reset_slew()

    def teleopPeriodic(self):
        self.drive_train.swerveDrive(self.joystick)
        self.arm_controller.setArm(self.joystick)

if __name__ == '__main__':
    wpilib.run(Robot)