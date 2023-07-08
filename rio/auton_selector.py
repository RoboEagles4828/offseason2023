import wpilib
from hardware_interface.armcontroller import ArmController
from hardware_interface.drivetrain import DriveTrain
import time
from wpilib import Timer

class AutonSelector():
    def __init__(self, arm_controller: ArmController, drive_train: DriveTrain):
        self.arm_controller = arm_controller
        self.drive_train = drive_train
        self.TAXI = "Taxi Auton"
        self.HIGH_PLACE = "High Place Auton"
        self.HIGH_TAXI = "High Taxi Auton"
        self.CHARGE = "Charge Auton"
        self.autonChooser = wpilib.SendableChooser()
        self.autonChooser.setDefaultOption("Taxi Auton", self.TAXI)
        self.autonChooser.addOption("High Place Auton", self.HIGH_PLACE)
        self.autonChooser.addOption("High Taxi Auton", self.HIGH_TAXI)
        self.autonChooser.addOption("Charge Auton", self.CHARGE)

        self.selected = self.autonChooser.getSelected()

        self.timer = Timer()
        self.start = 0

    def run(self):
        self.timer.start()
        self.selected = self.autonChooser.getSelected()
        if self.selected == self.TAXI:
            self.taxi_auton()
            self.timer.stop()
        elif self.selected == self.HIGH_PLACE: 
            self.timer.start()
            self.high_place_auton()
            self.timer.stop()
        elif self.selected == self.HIGH_TAXI:
            self.timer.start()
            self.high_taxi_auton()
            self.timer.stop()
        elif self.selected == self.CHARGE:
            self.timer.reset()
            self.charge_auton()
            self.timer.stop()

    def set_start_time(self, time):
        self.start = time

    def high_place_auton(self):
        if  0 <= self.timer.getFPGATimestamp() - self.start < 1:
            print(f"roller up {self.timer.getFPGATimestamp() - self.start}")
            self.arm_controller.elevator_pivot_control(1)
        elif 1 <= self.timer.getFPGATimestamp() - self.start < 3:
            print(f"elevator high {self.timer.getFPGATimestamp() - self.start}")
            self.arm_controller.elevator_high_level(1)
        elif 3 <= self.timer.getFPGATimestamp() - self.start < 4:
            print(f"release {self.timer.getFPGATimestamp() - self.start}")
            self.arm_controller.top_gripper_control(0)
        elif 4 <= self.timer.getFPGATimestamp() - self.start < 5:
            print(f"elevator reset {self.timer.getFPGATimestamp() - self.start}")
            self.arm_controller.elevator.setPosition(self.arm_controller.elevator.min)
        elif 5 <= self.timer.getFPGATimestamp() - self.start < 5.5:
            print(f"roller down {self.timer.getFPGATimestamp() - self.start}")
            self.arm_controller.arm_roller_bar.setPosition(self.arm_controller.arm_roller_bar.min)

    def taxi_auton(self):
        if self.timer.getFPGATimestamp() - self.start < 8.5:
            print(f"Taxi Auton {self.timer.getFPGATimestamp() - self.start}")
            self.drive_train.swerveDriveAuton(0, 1, 0)
        elif self.timer.getFPGATimestamp() - self.start < 9:
            print("Taxi Auton Stop")
            self.drive_train.stop()

    def high_taxi_auton(self):
        if self.timer.getFPGATimestamp() - self.start > 5.5:
            self.taxi_auton()
        else:
            self.high_place_auton()

    def charge_auton(self):
        print("Charge Auton")
        # TODO: Implement
        pass

    def timer_reset(self):
        self.timer.reset()
    


