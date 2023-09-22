import wpilib
import logging

CONTROLLER_PORT = 0
SCALING_FACTOR_FIX = 1

ENABLE_THROTTLE = False

pov_x_map = {
    -1: 0.0,
    0: 0.0,
    45: -1.0,
    90: -1.0,
    135: -1.0,
    180: 0.0,
    225: 1.0,
    270: 1.0,
    315: 1.0,
}

pov_y_map = {
    -1: 0.0,
    0: 1.0,
    45: 1.0,
    90: 0.0,
    135: -1.0,
    180: -1.0,
    225: -1.0,
    270: 0.0,
    315: 1.0,
}

class Joystick:
    def __init__(self, type: str):
        self.type = type.lower()
        if self.type.lower() == "xbox":
            self.joystick = wpilib.XboxController(CONTROLLER_PORT)
        elif self.type.lower() == "ps4":
            self.joystick = wpilib.PS4Controller(CONTROLLER_PORT)
        self.deadzone = 0.1
        self.last_joystick_data = self.getEmptyData()
        self.count = 0

    def scaleAxis(self, axis):
        return axis * SCALING_FACTOR_FIX * -1.0
    
    def scaleTrigger(self, trigger):
        return trigger * SCALING_FACTOR_FIX
    
    def getEmptyData(self):
        return {
            "axes": [0.0] * 8,
            "buttons": [0] * 11,
        }
    
    def getData(self):
        if self.type.lower() == "xbox":
            self.joystick = wpilib.XboxController(CONTROLLER_PORT)
        elif self.type.lower() == "ps4":
            self.joystick = wpilib.PS4Controller(CONTROLLER_PORT)
        pov = self.joystick.getPOV()
        leftX = self.joystick.getLeftX()
        leftY = self.joystick.getLeftY()
        rightX = self.joystick.getRightX()
        rightY = self.joystick.getRightY()
        leftTrigger = self.joystick.getLeftTriggerAxis() if self.type == "xbox" else self.joystick.getL2Axis()
        rightTrigger = self.joystick.getRightTriggerAxis() if self.type == "xbox" else self.joystick.getR2Axis()

        axes = [
            self.scaleAxis(leftX) if abs(leftX) > self.deadzone else 0.0, # 0
            self.scaleAxis(leftY) if abs(leftY) > self.deadzone else 0.0, # 1
            self.scaleTrigger(leftTrigger), # 2
            self.scaleAxis(rightX) if abs(rightX) > self.deadzone else 0.0, # 3
            self.scaleAxis(rightY) if abs(rightY) > self.deadzone else 0.0, # 4
            self.scaleTrigger(rightTrigger), # 5
            pov_x_map[pov], # 6
            pov_y_map[pov], # 7
        ]
        if self.type == "xbox":
            buttons = [
                int(self.joystick.getAButton()), # 0
                int(self.joystick.getBButton()), # 1
                int(self.joystick.getXButton()), # 2
                int(self.joystick.getYButton()), # 3
                int(self.joystick.getLeftBumper()), # 4
                int(self.joystick.getRightBumper()), # 5
                int(self.joystick.getBackButton()), # 6
                int(self.joystick.getStartButton()), # 7
                0, # 8
                int(self.joystick.getLeftStickButton()), # 9
                int(self.joystick.getRightStickButton()) # 10
            ]
        else:
            buttons = [
                int(self.joystick.getCrossButton()), # 0
                int(self.joystick.getCircleButton()), # 1
                int(self.joystick.getSquareButton()), # 2
                int(self.joystick.getTriangleButton()), # 3
                int(self.joystick.getL1Button()), # 4
                int(self.joystick.getR1Button()), # 5
                int(self.joystick.getShareButton()), # 6
                int(self.joystick.getOptionsButton()), # 7
                0, # 8
                int(self.joystick.getL3Button()), # 9
                int(self.joystick.getR3Button()) # 10
            ]

        data = {"axes": axes, "buttons": buttons}

        if ENABLE_THROTTLE:
            if self.is_equal(data, self.last_joystick_data):
                self.count += 1
                if self.count >= 10:
                    return None
            else:
                self.count = 0
                self.last_joystick_data = data
                return data
        else:
            return data

    def is_equal(self, d, d1):
        res = all((d1.get(k) == v for k, v in d.items()))
        return res
    
    

