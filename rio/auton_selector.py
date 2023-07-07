import wpilib
from hardware_interface.armcontroller import ArmController
from hardware_interface.drivetrain import DriveTrain
import time

class AutonSelector():
    def __init__(self, arm_controller: ArmController, drive_train: DriveTrain):
        self.arm_controller = arm_controller
        self.drive_train = drive_train
        self.TAXI = "Taxi Auton"
        self.HIGH_PLACE = "High Place Auton"
        self.HIGH_TAXI = "High Taxi Auton"
        self.CHARGE = "Charge Auton"
        self.autonChooser = wpilib.SendableChooser()
        self.autonChooser.setDefaultOption("Taxi Auton", self.taxi_auton)
        self.autonChooser.addOption("High Place Auton", self.high_place_auton)
        self.autonChooser.addOption("High Taxi Auton", self.high_taxi_auton)
        self.autonChooser.addOption("Charge Auton", self.charge_auton)


    def high_place_auton(self):
        self.arm_controller.elevator_pivot_control(1)
        time.sleep(1)
        self.arm_controller.elevator_high_level(1)
        time.sleep(2)
        self.arm_controller.top_gripper_control(0)
        time.sleep(1)
        self.arm_controller.arm_roller_bar.setPosition(self.arm_controller.arm_roller_bar.min)
        time.sleep(0.5)
        self.arm_controller.elevator.setPosition(self.arm_controller.elevator.min)

    def taxi_auton(self):
        self.drive_train.swerveDriveAuton(0, 1, 0)
        time.sleep(5)
        self.drive_train.stop()

    def high_taxi_auton(self):
        self.high_place_auton()
        self.taxi_auton()

    def charge_auton(self):
        # TODO: Implement
        pass
    



