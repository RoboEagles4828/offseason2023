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
        self.HIGH_CHARGE = "High Charge Auton"
        self.TEST = "Test"
        self.autonChooser = wpilib.SendableChooser()
        self.autonChooser.addOption("Taxi Auton", self.TAXI)
        self.autonChooser.setDefaultOption("High Place Auton", self.HIGH_PLACE)
        self.autonChooser.addOption("High Taxi Auton", self.HIGH_TAXI)
        self.autonChooser.addOption("Charge Auton", self.CHARGE)
        self.autonChooser.addOption("High Charge Auton", self.HIGH_CHARGE)
        self.autonChooser.addOption("Test", self.TEST)

        self.selected = self.autonChooser.getSelected()

        self.timer = Timer()
        self.start = 0
        self.turn_done = False
        self.first_pitch = False

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
            self.set_start_time(self.timer.getFPGATimestamp())
            self.post_charge_auton()
            self.timer.stop()
        elif self.selected == self.TEST:
            self.test()
        elif self.selected == self.HIGH_CHARGE:
            self.timer.reset()
            self.high_place_auton()
            self.charge_auton()
            self.set_start_time(self.timer.getFPGATimestamp())
            self.post_charge_auton()
            self.timer.stop()

    def set_start_time(self, time):
        self.start = time

    def test(self):
        print("Pitch: " + str(self.drive_train.navx.getPitch()), end=" ")
        print("Roll: " + str(self.drive_train.navx.getRoll()), end=" ")
        print("Yaw: " + str(self.drive_train.navx.getYaw()), end=" ")
        print("Angle: " + str(self.drive_train.navx.getAngle()), end=" ")
        print("Rotation2D: " + str(self.drive_train.navx.getRotation2d().degrees()), end=" ")
        print("Fused: " + str(self.drive_train.navx.getFusedHeading()), end=" ")
        print("Compass: " + str(self.drive_train.navx.getCompassHeading()))

    def high_place_auton(self):
        if  0 <= self.timer.getFPGATimestamp() - self.start < 1:
            print(f"roller up {self.timer.getFPGATimestamp() - self.start}")
            self.arm_controller.arm_roller_bar.setPosition(self.arm_controller.arm_roller_bar.max)
        elif 1 <= self.timer.getFPGATimestamp() - self.start < 5:
            print(f"elevator high {self.timer.getFPGATimestamp() - self.start}")
            self.arm_controller.elevator_high_level_on()
        elif 5 <= self.timer.getFPGATimestamp() - self.start < 7:
            print(f"release {self.timer.getFPGATimestamp() - self.start}")
            self.arm_controller.top_gripper_control_off()
        elif 7 <= self.timer.getFPGATimestamp() - self.start < 10:
            print(f"elevator reset {self.timer.getFPGATimestamp() - self.start}")
            self.arm_controller.elevator_high_level_off()
        elif 10 <= self.timer.getFPGATimestamp() - self.start < 10.5:
            print(f"roller down {self.timer.getFPGATimestamp() - self.start}")
            self.arm_controller.elevator_pivot_control_off()

    def taxi_auton(self):
        if not self.turn_done:
            print(f"Taxi Turn {self.timer.getFPGATimestamp() - self.start}")
            self.drive_train.swerveDriveAuton(0, 0, 0.6)
            if self.drive_train.navx.getRotation2d().degrees()*-1 >= 180:
                self.turn_done = True
                self.drive_train.stop()
        elif self.timer.getFPGATimestamp() - self.start < 15:
            print(f"Taxi Auton {self.timer.getFPGATimestamp() - self.start}")
            self.drive_train.swerveDriveAuton(0, 0.6, 0)
        elif self.timer.getFPGATimestamp() - self.start >= 15:
            print("Taxi Auton Stop")
            self.drive_train.stop()

    def high_taxi_auton(self):
        if self.timer.getFPGATimestamp() - self.start > 10.5:
            self.taxi_auton()
        else:
            self.high_place_auton()

    def charge_auton(self):
        pitch = self.drive_train.navx.getPitch()
        if abs(pitch) < 10:
            self.drive_train.swerveDriveAuton(0, 0.6, 0)
        elif abs(pitch < 10):
            self.drive_train.stop()
            

    def post_charge_auton(self):
        if self.timer.getFPGATimestamp() - self.start < 2:
            self.drive_train.swerveDriveAuton(0, 0.3, 0)
        else:
            self.drive_train.stop()
        


    def timer_reset(self):
        self.timer.reset()
    



