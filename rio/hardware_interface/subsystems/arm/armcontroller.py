import wpilib
import wpilib.simulation
import ctre
import ctre.sensors
import time
import logging
import math
from hardware_interface.joystick import Joystick
from hardware_interface.toggle import ToggleButton
from hardware_interface.subsystems.arm.arm_constants import *
from hardware_interface.subsystems.arm.piston import Piston
from hardware_interface.subsystems.arm.elevator import Elevator
from hardware_interface.subsystems.arm.intake import Intake

class ArmController():
    def __init__(self):
        self.last_cmds_time = time.time()
        self.warn_timeout = True
        
        self.hub = wpilib.PneumaticHub(PORTS['HUB'])
        self.compressor = self.hub.makeCompressor()

        self.arm_roller_bar = Piston(self.hub, PORTS['ARM_ROLLER_BAR'], max=0.07, name="Arm Roller Bar")
        self.top_gripper_slider = Piston(self.hub, PORTS['TOP_GRIPPER_SLIDER'], max=0.30, name="Top Gripper Slider")
        self.top_gripper = Piston(self.hub, PORTS['TOP_GRIPPER'], min=0.0, max=-0.9, name="Top Gripper", reverse=True)
        
        self.servo = wpilib.Servo(0)

        self.elevator = Elevator(PORTS['ELEVATOR'], max=0.56)
        self.JOINT_MAP = {
            # Pneumatics
            'arm_roller_bar_joint': self.arm_roller_bar,
            'top_slider_joint': self.top_gripper_slider,
            'top_gripper_left_arm_joint': self.top_gripper,
            # Wheels
            'elevator_center_joint': self.elevator,
        }
        self.toggle_buttons = {
            "elevator_loading_station": ToggleButton(
                5, False, # Right Bumper
                self.elevator_loading_station_on, 
                self.elevator_loading_station_off
            ),

            "elevator_mid_level": ToggleButton(
                4, False, # Left Bumper
                self.elevator_mid_level_on,
                self.elevator_mid_level_off
            ),

            "elevator_high_level": ToggleButton(
                2, False, # X Button
                self.elevator_high_level_on,
                self.elevator_high_level_off
            ),

            "top_gripper_control": ToggleButton(
                0, False, # A Button
                self.top_gripper_control_on,
                self.top_gripper_control_off
            ),

            "elevator_pivot_control": ToggleButton(
                3, False, # Y Button
                self.elevator_pivot_control_on,
                self.elevator_pivot_control_off
            ),

            "top_slider_control": ToggleButton(
                1, False, # B Button
                self.top_slider_control_on,
                self.top_slider_control_off
            ),
            
            "servo_control": ToggleButton(
                6, True, # Left Trigger
                self.servo_control_on,
                self.servo_control_off
            )
        }
        self.toggle = False

    def setToggleButtons(self):
        self.toggle_buttons = {
            "elevator_loading_station": ToggleButton(
                5, False, # Right Bumper
                self.elevator_loading_station_on, 
                self.elevator_loading_station_off
            ),

            "elevator_mid_level": ToggleButton(
                4, False, # Left Bumper
                self.elevator_mid_level_on,
                self.elevator_mid_level_off
            ),

            "elevator_high_level": ToggleButton(
                2, False, # X Button
                self.elevator_high_level_on,
                self.elevator_high_level_off
            ),

            "top_gripper_control": ToggleButton(
                0, False, # A Button
                self.top_gripper_control_on,
                self.top_gripper_control_off
            ),

            "elevator_pivot_control": ToggleButton(
                3, False, # Y Button
                self.elevator_pivot_control_on,
                self.elevator_pivot_control_off
            ),

            "top_slider_control": ToggleButton(
                1, False, # B Button
                self.top_slider_control_on,
                self.top_slider_control_off
            ),
            
            "servo_control": ToggleButton(
                6, True, # Left Trigger
                self.servo_control_on,
                self.servo_control_off
            )
        }
        
    def getEncoderData(self):
        names = [""]*4
        positions = [0]*4
        velocities = [0]*4

        # Iterate over the JOINT_MAP and run the get() function for each of them
        for index, joint_name in enumerate(self.JOINT_MAP):
            names[index] = joint_name
            # Allow for joints to be removed quickly
            positions[index] = int(self.JOINT_MAP[joint_name].getPosition() * SCALING_FACTOR_FIX)
            velocities[index] = int(self.JOINT_MAP[joint_name].getVelocity() * SCALING_FACTOR_FIX)
        return { "name" : names, "position": positions, "velocity": velocities}

    def stop(self):
        for joint in self.JOINT_MAP.values():
            joint.stop()

    def sendCommands(self, commands):
        if commands:
            self.last_cmds_time = time.time()
            self.warn_timeout = True
            for i in range(len(commands["name"])):
                joint_name = commands["name"][i]
                if joint_name in self.JOINT_MAP:
                    # if joint_name != 'elevator_center_joint':
                    self.JOINT_MAP[joint_name].setPosition(commands['position'][i])
        
        elif (time.time() - self.last_cmds_time > CMD_TIMEOUT_SECONDS):
            self.stop()
            if self.warn_timeout:
                logging.warning(f"Didn't recieve any commands for {CMD_TIMEOUT_SECONDS} second(s). Halting...")
                self.warn_timeout = False

    def setArm(self, joystick: Joystick):
        for button in self.toggle_buttons.values():
            button.toggle(joystick.getData()["buttons"])
        # self.elevator.motor.set(ctre.ControlMode.PercentOutput, joystick.getData()["axes"][4])

    # Callback functions for toggle buttons (These were originally lambda functions inlined, but we decided this was more readable)
    def elevator_loading_station_on(self):
        self.elevator.setPosition(0.15)
        self.top_gripper_slider.setPosition(self.top_gripper_slider.max)

    def elevator_loading_station_off(self):
        self.elevator.setPosition(self.elevator.min)
        self.top_gripper_slider.setPosition(self.top_gripper_slider.min)

    def elevator_mid_level_on(self):
        self.elevator.setPosition(0.336)
        self.top_gripper_slider.setPosition(self.top_gripper_slider.max)

    def elevator_mid_level_off(self):
        self.elevator.setPosition(self.elevator.min)
        self.top_gripper_slider.setPosition(self.top_gripper_slider.min)
    
    def elevator_high_level_on(self):
        self.elevator.setPosition(0.35)
        self.top_gripper_slider.setPosition(self.top_gripper_slider.max)
    
    def elevator_high_level_off(self):
        self.elevator.setPosition(self.elevator.min)
        self.top_gripper_slider.setPosition(self.top_gripper_slider.min)

    def top_gripper_control_on(self):
        self.top_gripper.setPosition(self.top_gripper.min)

    def top_gripper_control_off(self):
        self.top_gripper.setPosition(self.top_gripper.max)
    
    def elevator_pivot_control_on(self):
        self.arm_roller_bar.setPosition(self.arm_roller_bar.max)

    def elevator_pivot_control_off(self):
        self.arm_roller_bar.setPosition(self.arm_roller_bar.min)

    def top_slider_control_on(self):
        self.top_gripper_slider.setPosition(self.top_gripper_slider.max)
    
    def top_slider_control_off(self):
        self.top_gripper_slider.setPosition(self.top_gripper_slider.min)
        
    def servo_control_on(self):
        self.servo.set(0.5)
    
    def servo_control_off(self):
        self.servo.set(0.0)