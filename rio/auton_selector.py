import wpilib
from hardware_interface.armcontroller import ArmController
from hardware_interface.drivetrain import DriveTrain
from hardware_interface.subsystems.drive_subsystem import DriveSubsystem
from hardware_interface.subsystems.arm_subsystem import ArmSubsystem
from hardware_interface.joystick import Joystick
from hardware_interface.commands.drive_commands import *
from hardware_interface.commands.arm_commands import *
import time


class AutonSelector():
    def __init__(self, arm_controller: ArmController, drive_train: DriveTrain, joystick: Joystick):
        self.arm_controller = arm_controller
        self.drive_train = drive_train
        self.joystick = joystick
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
        self.MID_TAXI_B = "Mid Taxi BUMP Auton"
        self.HIGH_TAXI_B = "High Taxi BUMP Auton"
        self.CUBE_HIGH_TAXI_B = "Cube High Taxi BUMP Auton"
        self.TAXI_AUTON_B = "Taxi Auton BUMP"
        self.autonChooser = wpilib.SendableChooser()
        self.autonChooser.setDefaultOption("Taxi CLEAN Auton", self.TAXI)
        self.autonChooser.addOption("Taxi BUMP Auton", self.TAXI_AUTON_B)
        self.autonChooser.addOption("High Place Auton", self.HIGH_PLACE)
        self.autonChooser.addOption("High Taxi CLEAN Auton", self.HIGH_TAXI)
        self.autonChooser.addOption("High Taxi BUMP Auton", self.HIGH_TAXI_B)
        # self.autonChooser.addOption("Cube High Taxi CLEAN Auton", self.CUBE_HIGH_TAXI)
        # self.autonChooser.addOption("Mid Place Auton", self.MID_PLACE)
        # self.autonChooser.addOption("Mid Taxi CLEAN Auton", self.MID_TAXI)
        # self.autonChooser.addOption("Mid Taxi BUMP Auton", self.MID_TAXI_B)
        # self.autonChooser.addOption("Cube High Taxi BUMP Auton", self.CUBE_HIGH_TAXI_B)
        self.autonChooser.addOption("Charge Auton", self.CHARGE)
        self.autonChooser.addOption("High Charge Auton", self.HIGH_CHARGE)
        # self.autonChooser.addOption("Cube High Place Auton", self.CUBE_HIGH_PLACE)
        # self.autonChooser.addOption("Mid Charge Auton", self.MID_CHARGE)

        self.selected = self.autonChooser.getSelected()

        self.start = 0
        self.turn_done = False
        self.first_pitch = False
        
        self.command = DoNothingCommand()
        
        self.drive_subsystem = DriveSubsystem(self.drive_train, self.joystick)
        self.arm_subsystem = ArmSubsystem(self.arm_controller)

    def run(self):
        self.selected = self.autonChooser.getSelected()
        # if self.selected == self.TAXI:
        #     self.command = self.taxi_auton("clean")
        # elif self.selected == self.HIGH_PLACE: 
        #     self.command = self.high_place_auton()
        # elif self.selected == self.HIGH_TAXI:
        #     self.command = self.high_taxi_auton("clean")
        # elif self.selected == self.CHARGE:
        #     self.command = self.charge_auton()
        # elif self.selected == self.CUBE_HIGH_TAXI:
        #     self.command = self.cube_high_taxi_auton("clean")
        # elif self.selected == self.HIGH_CHARGE:
        #     self.command = self.high_charge_auton()
        # elif self.selected == self.MID_TAXI:
        #     self.command = self.mid_taxi_auton("clean")
        # elif self.selected == self.CUBE_HIGH_PLACE:
        #     self.command = self.cube_high_place_auton()
        # elif self.selected == self.MID_PLACE:
        #     self.command = self.mid_place_auton()
        # elif self.selected == self.MID_CHARGE:
        #     self.command = self.mid_charge_auton()
        # elif self.selected == self.MID_TAXI_B:
        #     self.command = self.mid_taxi_auton("bump")
        
        autons = {
            self.TAXI: self.taxi_auton("clean"),
            self.TAXI_AUTON_B: self.taxi_auton("bump"),
            self.HIGH_PLACE: self.high_place_auton(),
            self.HIGH_TAXI: self.high_taxi_auton("clean"),
            self.HIGH_TAXI_B: self.high_taxi_auton("bump"),
            self.CHARGE: self.charge_auton(),
            self.HIGH_CHARGE: self.high_charge_auton(),
        }
        
        self.command = autons[self.selected]
        
        auton = self.command
        auton.schedule()

    def high_place_auton(self):
        high_place_auton = ScoreCommand(self.arm_subsystem, ElevatorState.HIGH, "cone")
        return high_place_auton
        
    # def mid_place_auton(self):
    #     mid_place_auton = ScoreCommand(self.arm_subsystem, ElevatorState.MID, "cone")
    #     return mid_place_auton

    # def mid_taxi_auton(self, side):
    #     mid_taxi_auton = SequentialCommandGroup(
    #         ScoreCommand(self.arm_subsystem, ElevatorState.MID, "cone"),
    #         TaxiAutoCommand(self.drive_subsystem, side)
    #     )
    #     return mid_taxi_auton

    def high_taxi_auton(self, side):
        high_taxi_auton = SequentialCommandGroup(
            ScoreCommand(self.arm_subsystem, ElevatorState.HIGH, "cone"),
            TaxiAutoCommand(self.drive_subsystem, side)
        )
        return high_taxi_auton

    def taxi_auton(self, side):
        taxiAuton = TaxiAutoCommand(self.drive_subsystem, side)
        return taxiAuton    
    
    def charge_auton(self):
        chargeAuton = SequentialCommandGroup(
            DriveToChargeStationCommand(self.drive_subsystem, 10),
            BalanceOnChargeStationCommand(self.drive_subsystem, 7)
        )
        return chargeAuton
        
    def high_charge_auton(self):
        high_charge_auton = SequentialCommandGroup(
            ScoreCommand(self.arm_subsystem, ElevatorState.HIGH, "cone"),
            DriveToChargeStationCommand(self.drive_subsystem, 10),
            BalanceOnChargeStationCommand(self.drive_subsystem, 7)
        )
        return high_charge_auton


        
    



