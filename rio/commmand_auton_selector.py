from commands2 import *
import wpilib
from commands.base_commands.drive_cmds import *
from commands.base_commands.elevator_cmds import *
from commands.base_commands.piston_cmds import *
from hardware_interface.armcontroller import ArmController
from hardware_interface.drivetrain import DriveTrain

class CommandAutonSelector():
    def __init__(self, arm_controller: ArmController, drive_train: DriveTrain):
        self.scheduler = CommandScheduler.getInstance()
        self.arm_controller = arm_controller
        self.drive_train = drive_train
        self.TAXI = "Taxi Auton"
        self.HIGH_PLACE = "High Place Auton"
        self.HIGH_TAXI = "High Taxi Auton"
        self.CHARGE = "Charge Auton"
        self.HIGH_CHARGE = "High Charge Auton"
        self.MID_TAXI = "Mid Taxi Auton"
        self.CUBE_HIGH_TAXI = "Cube High Taxi Auton"
        self.MID_PLACE = "Mid Place Auton"
        self.autonChooser = wpilib.SendableChooser()
        self.autonChooser.addOption("Taxi Auton", self.TAXI)
        self.autonChooser.setDefaultOption("High Place Auton", self.HIGH_PLACE)
        self.autonChooser.addOption("High Taxi Auton", self.HIGH_TAXI)
        self.autonChooser.addOption("Cube High Taxi Auton", self.CUBE_HIGH_TAXI)
        self.autonChooser.addOption("Mid Taxi Auton", self.MID_TAXI)
        self.autonChooser.addOption("Mid Place Auton", self.MID_PLACE)
        
        self.selected = self.autonChooser.getSelected()
        
        self.first_pitch = False
        
    def run(self):
        self.selected = self.autonChooser.getSelected()
        if self.selected == self.TAXI:
            if self.taxi_auton() != None:
                self.scheduler.schedule(self.taxi_auton())
        elif self.selected == self.HIGH_PLACE: 
            # if self.high_place_auton() != None:
            self.high_place_auton()
        elif self.selected == self.HIGH_TAXI:
            if self.high_taxi_auton() != None:
                self.scheduler.schedule(self.high_taxi_auton())
        elif self.selected == self.CHARGE:
            self.charge_auton()
        elif self.selected == self.CUBE_HIGH_TAX:
            if self.cube_high_taxi_auton() != None:
                self.scheduler.schedule(self.cube_high_taxi_auton())
        elif self.selected == self.HIGH_CHARGE:
            if self.high_place_auton() != None:
                self.scheduler.schedule(self.high_place_auton())
            if self.high_place_auton().isFinished():
                self.charge_auton()
        elif self.selected == self.MID_PLACE:
            if self.mid_place_auton() != None:
                self.scheduler.schedule(self.mid_place_auton())
        elif self.selected == self.MID_TAXI:
            if self.mid_taxi_auton() != None:
                self.scheduler.schedule(self.mid_taxi_auton())
    
    def high_place_auton(self):
        PivotUpCommand(self.arm_controller).withTimeout(0.5).andThen(
        WaitCommand(0.5).withTimeout(0.5)).andThen(
        ElevatorHighCommand(self.arm_controller).withTimeout(3)).andThen(
        WaitCommand(0.5).withTimeout(0.5)).andThen(
        ReleaseCommand(self.arm_controller).withTimeout(1.5)).andThen(
        WaitCommand(1.5).withTimeout(1.5)).andThen(
        ElevatorRetractCommand(self.arm_controller).withTimeout(3)).andThen(
        WaitCommand(3).withTimeout(3)).andThen(
        PivotDownCommand(self.arm_controller).withTimeout(0.5)).schedule()
        
    def taxi_auton(self):
        return MoveCommand(self.drive_train, -4, 0.7, 0, 0)

    def high_taxi_auton(self):
        return SequentialCommandGroup(
            self.high_place_auton(),
            self.taxi_auton()
        )
    
    def mid_place_auton(self):
        return SequentialCommandGroup(
            ElevatorMidCommand(self.arm_controller),
            WaitCommand(0.5),
            ReleaseCommand(self.arm_controller),
            WaitCommand(1.5),
            ElevatorRetractCommand(self.arm_controller),
        )
        
    def mid_taxi_auton(self):
        return SequentialCommandGroup(
            self.mid_place_auton(),
            self.taxi_auton()
        )
        
    def cube_high_place_auton(self):
        return SequentialCommandGroup(
            ElevatorHighCommand(self.arm_controller),
            WaitCommand(0.5),
            ReleaseCommand(self.arm_controller),
            WaitCommand(1.5),
            ElevatorRetractCommand(self.arm_controller),
        )
            
    def cube_high_taxi_auton(self):
        return SequentialCommandGroup(
            self.cube_high_place_auton(),
            self.taxi_auton()
        )
        
    def charge_auton(self):
        pitch = self.drive_train.navx.getPitch()
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
            
            
            