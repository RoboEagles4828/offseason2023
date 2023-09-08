import wpilib
from hardware_interface.armcontroller import ArmController
from hardware_interface.drivetrain import DriveTrain
from hardware_interface.subsystems.drive_subsystem import DriveSubsystem
from hardware_interface.subsystems.arm_subsystem import ArmSubsystem
from hardware_interface.commands.drive_commands import *
from hardware_interface.commands.arm_commands import *
import time


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
        self.CUBE_HIGH_PLACE = "Cube High Place Auton"
        self.MID_PLACE = "Mid Place Auton"
        self.MID_CHARGE = "Mid Charge Auton"
        self.autonChooser = wpilib.SendableChooser()
        self.autonChooser.addOption("Taxi Auton", self.TAXI)
        self.autonChooser.addOption("High Place Auton", self.HIGH_PLACE)
        self.autonChooser.setDefaultOption("High Taxi Auton", self.HIGH_TAXI)
        self.autonChooser.addOption("Cube High Taxi Auton", self.CUBE_HIGH_TAXI)
        self.autonChooser.addOption("Mid Taxi Auton", self.MID_TAXI)
        self.autonChooser.addOption("Charge Auton", self.CHARGE)
        self.autonChooser.addOption("High Charge Auton", self.HIGH_CHARGE)
        self.autonChooser.addOption("Cube High Place Auton", self.CUBE_HIGH_PLACE)
        self.autonChooser.addOption("Mid Place Auton", self.MID_PLACE)
        self.autonChooser.addOption("Mid Charge Auton", self.MID_CHARGE)

        self.selected = self.autonChooser.getSelected()

        self.start = 0
        self.turn_done = False
        self.first_pitch = False
        
        self.command = DoNothingCommand()
        
        self.drive_subsystem = DriveSubsystem(self.drive_train)
        self.arm_subsystem = ArmSubsystem(self.arm_controller)

    def run(self):
        self.selected = self.autonChooser.getSelected()
        if self.selected == self.TAXI:
            self.command = self.taxi_auton()
        elif self.selected == self.HIGH_PLACE: 
            self.command = self.high_place_auton()
        elif self.selected == self.HIGH_TAXI:
            self.command = self.high_taxi_auton()
        elif self.selected == self.CHARGE:
            self.command = self.charge_auton()
        elif self.selected == self.CUBE_HIGH_TAXI:
            self.command = self.cube_high_taxi_auton()
        elif self.selected == self.HIGH_CHARGE:
            self.command = self.high_charge_auton()
        elif self.selected == self.MID_TAXI:
            self.command = self.mid_taxi_auton()
        elif self.selected == self.CUBE_HIGH_PLACE:
            self.command = self.cube_high_place_auton()
        elif self.selected == self.MID_PLACE:
            self.command = self.mid_place_auton()
        elif self.selected == self.MID_CHARGE:
            self.command = self.mid_charge_auton()
            
        auton = self.command
        auton.schedule()
            
    def cube_high_taxi_auton(self):
        cube_high_taxi_auton = SequentialCommandGroup(
            ScoreCommand(self.arm_subsystem, ElevatorState.HIGH, "cube"),
            TaxiAutoCommand(self.drive_subsystem)
        )
        return cube_high_taxi_auton
        
    def cube_high_place_auton(self):
        cube_high_place_auton = ScoreCommand(self.arm_subsystem, ElevatorState.HIGH, "cube")
        return cube_high_place_auton

    def high_place_auton(self):
        high_place_auton = ScoreCommand(self.arm_subsystem, ElevatorState.HIGH, "cone")
        return high_place_auton
        
    def mid_place_auton(self):
        mid_place_auton = ScoreCommand(self.arm_subsystem, ElevatorState.MID, "cone")
        return mid_place_auton

    def mid_taxi_auton(self):
        mid_taxi_auton = SequentialCommandGroup(
            ScoreCommand(self.arm_subsystem, ElevatorState.MID, "cone"),
            TaxiAutoCommand(self.drive_subsystem)
        )
        return mid_taxi_auton

    def high_taxi_auton(self):
        high_taxi_auton = SequentialCommandGroup(
            ScoreCommand(self.arm_subsystem, ElevatorState.HIGH, "cone"),
            TaxiAutoCommand(self.drive_subsystem)
        )
        return high_taxi_auton

    def taxi_auton(self):
        taxiAuton = TaxiAutoCommand(self.drive_subsystem)
        return taxiAuton    
    
    def charge_auton(self):
        chargeAuton = SequentialCommandGroup(
            DriveToChargeStationCommand(self.drive_subsystem, 10),
            BalanceOnChargeStationCommand(self.drive_subsystem, 0)
        )
        return chargeAuton
        
    def high_charge_auton(self):
        high_charge_auton = SequentialCommandGroup(
            ScoreCommand(self.arm_subsystem, ElevatorState.HIGH, "cone"),
            DriveToChargeStationCommand(self.drive_subsystem, 10),
            BalanceOnChargeStationCommand(self.drive_subsystem, 5)
        )
        return high_charge_auton

    def mid_charge_auton(self):
        mid_charge_auton = SequentialCommandGroup(
            ScoreCommand(self.arm_subsystem, ElevatorState.MID, "cone"),
            DriveToChargeStationCommand(self.drive_subsystem, 10),
            BalanceOnChargeStationCommand(self.drive_subsystem, 0)
        )
        return mid_charge_auton


        
    



