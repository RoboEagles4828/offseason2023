
import wpilib
import wpilib.simulation
import ctre
from pyfrc.physics import drivetrains
from pyfrc.physics.core import PhysicsInterface

from sim.talonFxSim import TalonFxSim
from sim.cancoderSim import CancoderSim
from hardware_interface.drivetrain import getAxleRadians, getWheelRadians, SwerveModule, AXLE_JOINT_GEAR_RATIO
from hardware_interface.armcontroller import PORTS, TOTAL_INTAKE_REVOLUTIONS
from hardware_interface.joystick import CONTROLLER_PORT

import math
import typing

if typing.TYPE_CHECKING:
    from rio.ros2robot import EdnaRobot
    
import threading
import logging
import time
import traceback
import os
import inspect

from dds.dds import DDS_Subscriber

# Calculations
axle_radius = 0.05
axle_mass = 0.23739
wheel_radius = 0.0508
wheel_length = 0.0381
wheel_mass = 0.2313

center_axle_moi = 0.5 * pow(axle_radius, 2) * axle_mass
center_side_wheel_moi = (0.25 * pow(wheel_radius, 2) * wheel_mass) + ((1/12) * pow(wheel_length, 2) * wheel_mass)
center_wheel_moi = 0.5 * pow(wheel_radius, 2) * wheel_mass

curr_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
xml_path = os.path.join(curr_path, "dds/xml/ROS_RTI.xml")

rti_init_lock = threading.Lock()
stop_threads=False

ISAAC_PARTICIPANT_NAME = "ROS2_PARTICIPANT_LIB::isaac_subscriber"
ISAAC_READER_NAME = "isaac_joint_states_subscriber::isaac_joint_states_reader"

class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "EdnaRobot"):
        
        self.physics_controller = physics_controller
        
        self.xbox = wpilib.simulation.XboxControllerSim(CONTROLLER_PORT)
        self.roborio = wpilib.simulation.RoboRioSim()
        self.battery = wpilib.simulation.BatterySim()
        self.roborio.setVInVoltage(self.battery.calculate([0.0]))

        self.frontLeftModuleSim = SwerveModuleSim(robot.drive_train.front_left)
        self.frontRightModuleSim = SwerveModuleSim(robot.drive_train.front_right)
        self.rearLeftModuleSim = SwerveModuleSim(robot.drive_train.rear_left)
        self.rearRightModuleSim = SwerveModuleSim(robot.drive_train.rear_right)

        self.elevator = TalonFxSim(robot.arm_controller.elevator.motor, 0.0003, 1, False)
        # self.intake = TalonFxSim(robot.arm_controller.bottom_gripper_lift.motor, 0.0004, 1, False)
        # self.intake.addLimitSwitch("fwd", 0)
        # self.intake.addLimitSwitch("rev", TOTAL_INTAKE_REVOLUTIONS * -2 * math.pi)

        self.pneumaticHub = wpilib.simulation.REVPHSim(PORTS['HUB'])
        self.armRollerBar = wpilib.simulation.DoubleSolenoidSim(self.pneumaticHub, *PORTS['ARM_ROLLER_BAR'])
        self.topGripperSlider = wpilib.simulation.DoubleSolenoidSim(self.pneumaticHub, *PORTS['TOP_GRIPPER_SLIDER'])
        self.topGripper = wpilib.simulation.DoubleSolenoidSim(self.pneumaticHub, *PORTS['TOP_GRIPPER'])
        
        self.isaac_thread = threading.Thread(target=self.isaacThread, daemon=True)
        self.isaac_thread.start()
        
        self.joint_state = None
        
        self.front_left_wheel = 0
        self.front_right_wheel = 0
        self.rear_left_wheel = 0
        self.rear_right_wheel = 0
        
        self.front_left_axle = 0
        self.front_right_axle = 0
        self.rear_left_axle = 0
        self.rear_right_axle = 0
        
        self.empty = {
            "wheel": [
                0,
                0
            ],
            "axle": [
                0,
                0
            ]
        }
        
        self.empty_list = [0, 0]
        
        self.front_left_state = self.empty
        self.front_right_state = self.empty
        self.rear_left_state = self.empty
        self.rear_right_state = self.empty
        
        self.elevator_state = self.empty_list
        
    def initDDS(self, ddsAction, participantName, actionName):
        dds = None
        with rti_init_lock:
            dds = ddsAction(xml_path, participantName, actionName)
        return dds

    def threadLoop(self, name, dds, action):
        logging.info(f"Starting {name} thread")
        global stop_threads
        try:
            while stop_threads == False:
                action(dds)
                time.sleep(20/1000)
        except Exception as e:
            logging.error(f"An issue occured with the {name} thread")
            logging.error(e)
            logging.error(traceback.format_exc())
        
        logging.info(f"Closing {name} thread")
        dds.close()

    def update_sim(self, now: float, tm_diff: float) -> None:
        
        # Get swerve module_indicies
        if self.joint_state != None:
            self.front_left_wheel = self.joint_state["name"].index("front_left_wheel_joint")
            self.front_right_wheel = self.joint_state["name"].index("front_right_wheel_joint")
            self.rear_left_wheel = self.joint_state["name"].index("rear_left_wheel_joint")
            self.rear_right_wheel = self.joint_state["name"].index("rear_right_wheel_joint")
            
            self.front_left_axle = self.joint_state["name"].index("front_left_axle_joint")
            self.front_right_axle = self.joint_state["name"].index("front_right_axle_joint")
            self.rear_left_axle = self.joint_state["name"].index("rear_left_axle_joint")
            self.rear_right_axle = self.joint_state["name"].index("rear_right_axle_joint")
            
            self.elevator_center = self.joint_state["name"].index("elevator_center_joint")
            
            self.elevator_state = [
                self.joint_state["position"][self.elevator_center],
                self.joint_state["velocity"][self.elevator_center]
            ]
            
            self.front_left_state = {
                "wheel": [
                    self.joint_state["position"][self.front_left_wheel],
                    self.joint_state["velocity"][self.front_left_wheel]
                ],
                "axle": [
                    self.joint_state["position"][self.front_left_axle],
                    self.joint_state["velocity"][self.front_left_axle]
                ]
            }
            
            self.front_right_state = {
                "wheel": [
                    self.joint_state["position"][self.front_right_wheel],
                    self.joint_state["velocity"][self.front_right_wheel]
                ],
                "axle": [
                    self.joint_state["position"][self.front_right_axle],
                    self.joint_state["velocity"][self.front_right_axle]
                ]
            }
            
            self.rear_left_state = {
                "wheel": [
                    self.joint_state["position"][self.rear_left_wheel],
                    self.joint_state["velocity"][self.rear_left_wheel]
                ],
                "axle": [
                    self.joint_state["position"][self.rear_left_axle],
                    self.joint_state["velocity"][self.rear_left_axle]
                ]
            }
            
            self.rear_right_state = {
                "wheel": [
                    self.joint_state["position"][self.rear_right_wheel],
                    self.joint_state["velocity"][self.rear_right_wheel]
                ],
                "axle": [
                    self.joint_state["position"][self.rear_right_axle],
                    self.joint_state["velocity"][self.rear_right_axle]
                ]
            }     
                
        # logging.info(self.joint_state)   
        
        # Simulate Swerve Modules
        self.frontLeftModuleSim.update(tm_diff, self.front_left_state["wheel"], self.front_left_state["axle"], True)
        self.frontRightModuleSim.update(tm_diff, self.front_right_state["wheel"], self.front_right_state["axle"], True)
        self.rearLeftModuleSim.update(tm_diff, self.rear_left_state["wheel"], self.rear_left_state["axle"], True)
        self.rearRightModuleSim.update(tm_diff, self.rear_right_state["wheel"], self.rear_right_state["axle"], True)

        # Simulate Arm
        self.elevator.update(tm_diff, 0, 0, False)
        # self.intake.update(tm_diff)

        # Add Currents into Battery Simulation
        self.roborio.setVInVoltage(self.battery.calculate([0.0]))
        
    def isaacThread(self):
        isaac_subscriber = self.initDDS(DDS_Subscriber, ISAAC_PARTICIPANT_NAME, ISAAC_READER_NAME)
        self.threadLoop("isaac", isaac_subscriber, self.isaacAction)
        
    def isaacAction(self, subscriber):
        self.joint_state = subscriber.read()
        # logging.info(self.isaac_thread.is_alive())

class SwerveModuleSim():
    wheel : TalonFxSim = None
    axle : TalonFxSim = None
    encoder : CancoderSim = None

    def __init__(self, module: "SwerveModule"):
        wheelMOI = center_wheel_moi
        axleMOI = center_axle_moi + center_side_wheel_moi
        self.wheel = TalonFxSim(module.wheel_motor, wheelMOI, 1, False)
        self.axle = TalonFxSim(module.axle_motor, axleMOI, AXLE_JOINT_GEAR_RATIO, False)
        self.encoder = CancoderSim(module.encoder, module.encoder_offset, True)
        # There is a bad feedback loop between controller and the rio code
        # It will create oscillations in the simulation when the robot is not being commanded to move
        # The issue is from the controller commanding the axle position to stay at the same position when idle
        # but if the axle is moving during that time it will constantly overshoot the idle position
    
    def update(self, tm_diff, wheel_state, axle_state, use_isaac):
        self.wheel.update(tm_diff, wheel_state[0], wheel_state[1], use_isaac)
        self.axle.update(tm_diff, axle_state[0], axle_state[1], use_isaac)
        self.encoder.update(tm_diff, axle_state[1])
    
    # Useful for debugging the simulation or code
    def __str__(self) -> str:
        wheelPos = getWheelRadians(self.wheel.talon.getSelectedSensorPosition(), "position")
        wheelVel = getWheelRadians(self.wheel.talon.getSelectedSensorVelocity(), "velocity")
        stateStr = f"Wheel POS: {wheelPos:5.2f} VEL: {wheelVel:5.2f} "
        
        axlePos = getAxleRadians(self.axle.talon.getSelectedSensorPosition(), "position")
        axleVel = getAxleRadians(self.axle.talon.getSelectedSensorVelocity(), "velocity")
        stateStr += f"Axle POS: {axlePos:5.2f} VEL: {axleVel:5.2f} "
        
        encoderPos = math.radians(self.encoder.cancoder.getAbsolutePosition())
        encoderVel = math.radians(self.encoder.cancoder.getVelocity())
        stateStr += f"Encoder POS: {encoderPos:5.2f} VEL: {encoderVel:5.2f}"
        return stateStr