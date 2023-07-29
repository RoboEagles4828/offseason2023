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
        self.MID_TAXI = "Mid Taxi Auton"
        self.CUBE_HIGH_TAXI = "Cube High Taxi Auton"
        self.autonChooser = wpilib.SendableChooser()
        self.autonChooser.addOption("Taxi Auton", self.TAXI)
        self.autonChooser.setDefaultOption("High Place Auton", self.HIGH_PLACE)
        self.autonChooser.addOption("High Taxi Auton", self.HIGH_TAXI)
        self.autonChooser.addOption("Cube High Taxi Auton", self.CUBE_HIGH_TAXI)
        self.autonChooser.addOption("Mid Taxi Auton", self.MID_TAXI)

        self.selected = self.autonChooser.getSelected()

        self.timer = Timer()
        self.start = 0
        self.turn_done = False
        self.first_pitch = False

    def run(self, auton=None):
        self.timer.start()
        if auton:
            self.selected = auton
        else:
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
        elif self.selected == self.CUBE_HIGH_TAXI:
            self.cube_high_taxi_auton()
        elif self.selected == self.HIGH_CHARGE:
            self.timer.reset()
            self.high_place_auton()
            self.charge_auton()
            self.post_charge_auton()
            self.timer.stop()
        elif self.selected == self.MID_TAXI:
            self.mid_taxi_auton()

    def set_start_time(self, time):
        self.start = time

    def cube_high_taxi_auton(self):
        if 0 <= self.timer.getFPGATimestamp() - self.start < 4:
            print(f"elevator high {self.timer.getFPGATimestamp() - self.start}")
            self.arm_controller.elevator_high_level_on()
        elif 4 <= self.timer.getFPGATimestamp() - self.start < 6:
            print(f"release {self.timer.getFPGATimestamp() - self.start}")
            self.arm_controller.top_gripper_control_off()
        elif 6 <= self.timer.getFPGATimestamp() - self.start < 9:
            print(f"elevator reset {self.timer.getFPGATimestamp() - self.start}")
            self.arm_controller.elevator_high_level_off()
        elif 9 <= self.timer.getFPGATimestamp() - self.start < 9+1.5:
            print(f"Taxi Auton {self.timer.getFPGATimestamp() - self.start}")
            self.drive_train.swerveDriveAuton(-0.8, 0, 0)
        elif self.timer.getFPGATimestamp() - self.start >= 9+1.5:
            self.drive_train.stop()

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

    def mid_taxi_auton(self):
        if  0 <= self.timer.getFPGATimestamp() - self.start < 3:
            self.arm_controller.elevator_mid_level_on()
        elif 3 <= self.timer.getFPGATimestamp() - self.start < 4:
            self.arm_controller.top_gripper_control_off()
        elif 4 <= self.timer.getFPGATimestamp() - self.start < 7:
            self.arm_controller.elevator_mid_level_off()
        elif 7 <= self.timer.getFPGATimestamp() - self.start < 7+1.5:
            print(f"Taxi Auton {self.timer.getFPGATimestamp() - self.start}")
            self.drive_train.swerveDriveAuton(-0.8, 0, 0)
        elif self.timer.getFPGATimestamp() - self.start >= 7+1.5:
            self.drive_train.stop()
        

    def high_taxi_auton(self):
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
        elif 10.5 <= self.timer.getFPGATimestamp() - self.start < 10.5+1.5:
            print(f"Taxi Auton {self.timer.getFPGATimestamp() - self.start}")
            self.drive_train.swerveDriveAuton(-0.8, 0, 0)
        elif self.timer.getFPGATimestamp() - self.start >= 10.5+1.5:
            self.drive_train.stop()


    def taxi_auton(self):
        # if not self.turn_done:
        #     print(f"Taxi Turn {self.timer.getFPGATimestamp() - self.start}")
        #     self.drive_train.swerveDriveAuton(0, 0, 0.6)
        #     if self.drive_train.navx.getRotation2d().__mul__(-1).degrees() >= 180:
        #         self.turn_done = True
        #         self.drive_train.stop()
        if self.timer.getFPGATimestamp() - self.start < 1.5:
            print(f"Taxi Auton {self.timer.getFPGATimestamp() - self.start}")
            self.drive_train.swerveDriveAuton(-0.8, 0, 0)
        elif self.timer.getFPGATimestamp() - self.start >= 1.5:
            print("Taxi Auton Stop")
            self.drive_train.stop()

    def charge_auton(self):
        pitch = self.drive_train.navx.getPitch()
        print(pitch)
        if not self.first_pitch:
            self.drive_train.swerveDriveAuton(-1.0, 0.0, 0.0)
            if pitch > 7:
                self.first_pitch = True
        else:
            if abs(pitch) < 5:
                self.drive_train.swerveDriveAuton(0, 0, 0)
                if pitch > 1:
                    self.drive_train.swerveDriveAuton(0.3, 0, 0)
                elif pitch < -1:
                    self.drive_train.swerveDriveAuton(-0.3, 0, 0)
            

    def post_charge_auton(self):
        if self.timer.getFPGATimestamp() - self.start < 2:
            self.drive_train.swerveDriveAuton(-0.3, 0, 0)
        else:
            self.drive_train.stop()


    def timer_reset(self):
        self.timer.reset()
    



