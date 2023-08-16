import wpilib
from hardware_interface.armcontroller import ArmController
from hardware_interface.drivetrain import DriveTrain
from hardware_interface.subsystems.drive_subsystem import DriveSubsystem
from hardware_interface.subsystems.arm_subsystem import ArmSubsystem
from hardware_interface.commands.drive_commands import *
from hardware_interface.commands.arm_commands import *
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
        self.autonChooser.addOption("High Place Auton", self.HIGH_PLACE)
        self.autonChooser.setDefaultOption("High Taxi Auton", self.HIGH_TAXI)
        self.autonChooser.addOption("Cube High Taxi Auton", self.CUBE_HIGH_TAXI)
        self.autonChooser.addOption("Mid Taxi Auton", self.MID_TAXI)

        self.selected = self.autonChooser.getSelected()

        self.timer = Timer()
        self.start = 0
        self.turn_done = False
        self.first_pitch = False
        
        self.drive_subsystem = DriveSubsystem(self.drive_train)
        self.arm_subsystem = ArmSubsystem(self.arm_controller)

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
        cube_high_taxi_auton = SequentialCommandGroup(
            ScoreCommand(self.arm_subsystem, ElevatorState.HIGH, "cube"),
            TurnToAngleCommand(self.drive_subsystem, 90),
        )
        cube_high_taxi_auton.schedule()

    def high_place_auton(self):
        high_place_auton = ScoreCommand(self.arm_subsystem, ElevatorState.HIGH, "cone")
        high_place_auton.schedule()

    def mid_taxi_auton(self):
        mid_taxi_auton = SequentialCommandGroup(
            ScoreCommand(self.arm_subsystem, ElevatorState.MID, "cone"),
            TaxiAutoCommand(self.drive_subsystem)
        )
        mid_taxi_auton.schedule()
        

    def high_taxi_auton(self):
        high_taxi_auton = SequentialCommandGroup(
            ScoreCommand(self.arm_subsystem, ElevatorState.HIGH, "cone"),
            TaxiAutoCommand(self.drive_subsystem)
        )
        high_taxi_auton.schedule()


    def taxi_auton(self):
        taxiAuton = TaxiAutoCommand(self.drive_subsystem)
        taxiAuton.schedule()

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
    



