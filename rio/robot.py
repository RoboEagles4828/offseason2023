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
        self.joystick = Joystick("xbox")
        self.auton_selector = AutonSelector(self.arm_controller, self.drive_train)
        self.joystick_selector = wpilib.SendableChooser()
        self.joystick_selector.setDefaultOption("XBOX", "xbox")
        self.joystick_selector.addOption("PS4", "ps4")
        self.auton_run = False

        self.shuffleboard = Shuffleboard.getTab("Main")
        self.shuffleboard.add(title="AUTON", defaultValue=self.auton_selector.autonChooser)

        self.shuffleboard.add(title="JOYSTICK", defaultValue=self.joystick_selector)

        self.shuffleboard.add("WHINE REMOVAL", self.drive_train.whine_remove_selector)

        self.shuffleboard.add("PROFILE", self.drive_train.profile_selector)
        self.shuffleboard.addDouble("YAW", lambda: (self.drive_train.navx.getRotation2d().degrees()))
        self.shuffleboard.addBoolean("FIELD ORIENTED", lambda: (self.drive_train.field_oriented_value))

        self.arm_controller.setToggleButtons()

    def robotPeriodic(self):
        self.joystick.type = self.joystick_selector.getSelected()

    def autonomousInit(self):
        self.auton_selector.timer_reset()
        self.auton_selector.set_start_time(self.auton_selector.timer.getFPGATimestamp())
        self.arm_controller.top_gripper_control_on()
        self.drive_train.navx.zeroYaw()

    def autonomousPeriodic(self):
        self.auton_selector.run()


    def teleopInit(self):
        self.arm_controller.setToggleButtons()
        self.drive_train.reset_slew()

    def teleopPeriodic(self):
        self.drive_train.swerveDrive(self.joystick)
        self.arm_controller.setArm(self.joystick)

if __name__ == '__main__':
    wpilib.run(Robot)