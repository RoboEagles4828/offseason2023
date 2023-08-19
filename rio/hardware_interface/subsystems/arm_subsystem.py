from commands2 import SubsystemBase
from hardware_interface.armcontroller import ArmController

class ElevatorState():
    HIGH = "high"
    MID = "mid"
    LOADING_STATION = "load"
    HOME = "home"
    
class PivotState():
    UP = True
    DOWN = False
    
class GrabberState():
    OPEN = False
    CLOSED = True

class ArmSubsystem(SubsystemBase):
    def __init__(self, armcontroller: ArmController):
        super().__init__()
        self.armcontroller = armcontroller
        
    def setElevator(self, preset : str):
        if preset.lower() == "high":
            self.armcontroller.elevator_high_level_on()
        elif preset.lower() == "mid":
            self.armcontroller.elevator_mid_level_on()
        elif preset.lower() in ["substation", "loading_station", "load", "sub"]:
            self.armcontroller.elevator_loading_station_on()
        elif preset.lower() in ["home", "reset", "zero"]:
            self.armcontroller.elevator_high_level_off()
            
        
    def setPivot(self, preset : bool):
        if preset:
            self.armcontroller.elevator_pivot_control_on()
        else:
            self.armcontroller.elevator_pivot_control_off()
            
    def setGrabber(self, preset : bool):
        if preset:
            self.armcontroller.top_gripper_control_on()
        else:
            self.armcontroller.top_gripper_control_off()
        
    def getEncoderData(self):
        return self.armcontroller.getEncoderData()
        
    def stop(self):
        self.armcontroller.stop()
        