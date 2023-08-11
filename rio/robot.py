from hardware_interface.drivetrain import DriveTrain
from hardware_interface.joystick import Joystick
from hardware_interface.armcontroller import ArmController
import wpilib
from wpilib.shuffleboard import Shuffleboard
from wpilib.shuffleboard import SuppliedFloatValueWidget
from auton_selector import AutonSelector
import time
from dds.dds import DDS_Publisher, DDS_Subscriber
import os
import inspect
import logging
import traceback
import threading
from networktables import NetworkTable, NetworkTables

ENABLE_STAGE_BROADCASTER = True
ENABLE_ENCODER = True
ENABLE_DASHBOARD = True

# Global Variables
arm_controller : ArmController = None
drive_train : DriveTrain = None
dashboard_data_list = list()
dashboard_data_return = dict()
frc_stage = "DISABLED"
fms_attached = False
stop_threads = False
nt : NetworkTable = None

# Logging
format = "%(asctime)s: %(message)s"
logging.basicConfig(format=format, level=logging.INFO, datefmt="%H:%M:%S")

# XML Path for DDS configuration
curr_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
xml_path = os.path.join(curr_path, "dds/xml/ROS_RTI.xml")

############################################
## Hardware
def initDriveTrain():
    global drive_train
    if drive_train == None:
        drive_train = DriveTrain()
        logging.info("Success: DriveTrain created")
    return drive_train

def initArmController():
    global arm_controller
    if arm_controller == None:
        arm_controller = ArmController()
        logging.info("Success: ArmController created")
    return arm_controller

def initDDS(ddsAction, participantName, actionName):
    dds = None
    with rti_init_lock:
        dds = ddsAction(xml_path, participantName, actionName)
    return dds

def initNetworkTable():
    global nt
    if nt == None:
        NetworkTables.initialize(server="localhost")
        nt = NetworkTables.getTable("Dashboard")
        logging.info("Success: NetworkTable created")
    return nt

############################################
## Threads
def threadLoop(name, dds, action):
    logging.info(f"Starting {name} thread")
    global stop_threads
    global frc_stage
    try:
        while stop_threads == False:
            if (frc_stage == 'AUTON' and name != "joystick") or (name in ["encoder", "stage-broadcaster", "dashboard", "dashboard_sub", "service"]) or (frc_stage == 'TELEOP'):
                action(dds)
            time.sleep(20/1000)
    except Exception as e:
        logging.error(f"An issue occured with the {name} thread")
        logging.error(e)
        logging.error(traceback.format_exc())
    
    logging.info(f"Closing {name} thread")
    dds.close()
    
# Generic Start Thread Function
def startThread(name) -> threading.Thread | None:
    thread = None
    if name == "encoder":
        thread = threading.Thread(target=encoderThread, daemon=True)
    elif name == "stage-broadcaster":
        thread = threading.Thread(target=stageBroadcasterThread, daemon=True)
    elif name == "dashboard":
        thread = threading.Thread(target=dashboardThread, daemon=True)
    elif name == "dashboard_sub":
        thread = threading.Thread(target=dashboardSubThread, daemon=True)
    elif name == "service":
        thread = threading.Thread(target=serviceThread, daemon=True)
    
    thread.start()
    return thread

# Locks
rti_init_lock = threading.Lock()
drive_train_lock = threading.Lock()
arm_controller_lock = threading.Lock()
############################################

################## ENCODER ##################
ENCODER_PARTICIPANT_NAME = "ROS2_PARTICIPANT_LIB::encoder_info"
ENCODER_WRITER_NAME = "encoder_info_publisher::encoder_info_writer"

def encoderThread():
    encoder_publisher = initDDS(DDS_Publisher, ENCODER_PARTICIPANT_NAME, ENCODER_WRITER_NAME)
    threadLoop('encoder', encoder_publisher, encoderAction)

def encoderAction(publisher):
    # TODO: Make these some sort of null value to identify lost data
    data = {
        'name': [],
        'position': [],
        'velocity': [],
        'effort': []
    }

    global drive_train
    with drive_train_lock:
        drive_data = drive_train.getModuleCommand()
        data['name'] += drive_data['name']
        data['position'] += drive_data['position']
        data['velocity'] += drive_data['velocity']
    
    global arm_controller
    with arm_controller_lock:
        arm_data = arm_controller.getEncoderData()
        data['name'] += arm_data['name']
        data['position'] += arm_data['position']
        data['velocity'] += arm_data['velocity']

    publisher.write(data)
############################################

################## STAGE ##################
STAGE_PARTICIPANT_NAME = "ROS2_PARTICIPANT_LIB::stage_broadcaster"
STAGE_WRITER_NAME = "stage_publisher::stage_writer"

def stageBroadcasterThread():
    stage_publisher = initDDS(DDS_Publisher, STAGE_PARTICIPANT_NAME, STAGE_WRITER_NAME)
    threadLoop('stage-broadcaster', stage_publisher, stageBroadcasterAction)

def stageBroadcasterAction(publisher : DDS_Publisher):
    global frc_stage
    global fms_attached
    is_disabled = wpilib.DriverStation.isDisabled()
    
    publisher.write({ "data": f"{frc_stage}|{fms_attached}|{is_disabled}" })
############################################

################## DASHBOARD ##################
DASHBOARD_PARTICIPANT_NAME = "ROS2_PARTICIPANT_LIB::dashboard"
DASHBOARD_WRITER_NAME = "dashboard_publisher::dashboard_writer"


def dashboardThread():
    dashboard_publisher = initDDS(DDS_Publisher, DASHBOARD_PARTICIPANT_NAME, DASHBOARD_WRITER_NAME)
    threadLoop('dashboard', dashboard_publisher, dashboardAction)
    
def dashboardAction(publisher : DDS_Publisher):
    global dashboard_data_list
    data = ""
    for i in dashboard_data_list:
        data += f"{str(i)}|"

    nt.putString("dashboard", data)
############################################

################## DASHBOARD SUB ###############
DASHBOARD_SUB = "ROS2_PARTICIPANT_LIB::dashboard_subscriber"
DASHBOARD_READER = "dashboard_data_subscriber::dashboard_reader"

def dashboardSubThread():
    dashboard_subscriber = initDDS(DDS_Subscriber, DASHBOARD_SUB, DASHBOARD_READER)
    threadLoop('dashboard_sub', dashboard_subscriber, dashboardSubAction)
    
def dashboardSubAction(subscriber : DDS_Subscriber):
    stuff = nt.getString("dashboard_sub", "str")
    if stuff:
        data = stuff
        dataArr = data.split("|")
        
        if len(dataArr) > 1:
            auton = dataArr[0]
            profile = dataArr[1]
            led = dataArr[2]
            joystick_type = dataArr[3]
            # service = dataArr[4]
            
            global dashboard_data_return
            dashboard_data_return = {
                "auton": auton,
                "profile": profile,
                "joy_type": joystick_type,
                # "service": service
            }
############################################   

class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.use_threading = True
        wpilib.CameraServer.launch()
        logging.warning("Running in simulation!") if wpilib.RobotBase.isSimulation() else logging.info("Running in real!")
        
        self.threads = []
        if self.use_threading:
            logging.info("Initializing Threads")
            global stop_threads
            stop_threads = False
            if ENABLE_ENCODER: self.threads.append({"name": "encoder", "thread": startThread("encoder") })
            if ENABLE_STAGE_BROADCASTER: self.threads.append({"name": "stage-broadcaster", "thread": startThread("stage-broadcaster") })
            if ENABLE_DASHBOARD: 
                self.threads.append({"name": "dashboard", "thread": startThread("dashboard") })
                self.threads.append({"name": "dashboard_sub", "thread": startThread("dashboard_sub") })
        else:
            self.encoder_publisher = DDS_Publisher(xml_path, ENCODER_PARTICIPANT_NAME, ENCODER_WRITER_NAME)
            self.stage_publisher = DDS_Publisher(xml_path, STAGE_PARTICIPANT_NAME, STAGE_WRITER_NAME)
            self.dashboard_publisher = DDS_Publisher(xml_path, DASHBOARD_PARTICIPANT_NAME, DASHBOARD_WRITER_NAME)
            self.dashboard_subscriber = DDS_Subscriber(xml_path, DASHBOARD_SUB, DASHBOARD_READER)
        
        self.arm_controller = initArmController()
        self.drive_train = initDriveTrain()
        self.nt = initNetworkTable()
        self.joystick = Joystick("ps4")
        self.auton_selector = AutonSelector(self.arm_controller, self.drive_train)
        self.joystick_selector = wpilib.SendableChooser()
        self.joystick_selector.setDefaultOption("XBOX", "xbox")
        self.joystick_selector.addOption("PS4", "ps4")
        self.auton_run = False
        
        self.profile = None
        self.auton = None

        self.shuffleboard = Shuffleboard.getTab("Main")
        self.shuffleboard.add(title="AUTON", defaultValue=self.auton_selector.autonChooser)

        self.shuffleboard.add(title="JOYSTICK", defaultValue=self.joystick_selector)

        self.shuffleboard.add("WHINE REMOVAL", self.drive_train.whine_remove_selector)
        self.shuffleboard.addDoubleArray("MOTOR VELOCITY", lambda: (self.drive_train.motor_vels))
        self.shuffleboard.addDoubleArray("MOTOR POSITIONS", lambda: (self.drive_train.motor_pos))

        self.shuffleboard.add("PROFILE", self.drive_train.profile_selector)
        self.shuffleboard.add("NAVX", self.drive_train.navx)
        self.shuffleboard.addDouble("YAW", lambda: (self.drive_train.navx.getYaw()))
        self.shuffleboard.addBoolean("FIELD ORIENTED", lambda: (self.drive_train.field_oriented_value))
        self.shuffleboard.addBoolean("SLOW", lambda: (self.drive_train.slow))
        self.shuffleboard.addDoubleArray("MOTOR TEMPS", lambda: (self.drive_train.motor_temps))
        self.shuffleboard.addDoubleArray("JOYSTICK OUTPUT", lambda: ([self.drive_train.linX, self.drive_train.linY, self.drive_train.angZ]))
        self.shuffleboard.addString("AUTO TURN STATE", lambda: (self.drive_train.auto_turn_value))

        self.arm_controller.setToggleButtons()

    def robotPeriodic(self):
        self.joystick.type = self.joystick_selector.getSelected()
        global dashboard_data_return
        if dashboard_data_return:
            self.profile = dashboard_data_return["profile"]
            self.auton = dashboard_data_return["auton"]
            joy_type = dashboard_data_return["joy_type"]
            self.joystick.type = joy_type
            
        global dashboard_data_list
        dashboard_data_list = self.drive_train.getDashboardData(
            self.joystick,
            self.auton_selector.autonChooser.getSelected(),
            self.joystick.joystick.isConnected(),
            float(self.arm_controller.compressor.getPressure()),
            f"{bool(self.arm_controller.top_gripper.spoof)}/{self.arm_controller.top_gripper.getPosition()}",
            f"{bool(self.arm_controller.top_gripper_slider.spoof)}/{self.arm_controller.top_gripper_slider.getPosition()}",
            f"{bool(self.arm_controller.arm_roller_bar.spoof)}/{self.arm_controller.arm_roller_bar.getPosition()}",
            float(self.arm_controller.elevator.getPosition()),
            float(wpilib.RobotController.getBatteryVoltage()),
        )


    # Auton
    def autonomousInit(self):
        self.auton_selector.timer_reset()
        self.auton_selector.set_start_time(self.auton_selector.timer.getFPGATimestamp())
        self.arm_controller.top_gripper_control_on()
        self.drive_train.navx.zeroYaw()
        logging.info("Entering Auton")
        global frc_stage
        frc_stage = "AUTON"
        

    def autonomousPeriodic(self):
        if self.auton:
            self.auton_selector.run(auton=self.auton)
        else:
            self.auton_selector.run()
        global fms_attached
        fms_attached = wpilib.DriverStation.isFMSAttached()
        if self.use_threading:
            self.manageThreads()
        else:
            self.doActions()
            
    def autonomousExit(self):
        logging.info("Exiting Auton")
        global frc_stage
        frc_stage = "AUTON"


    # Teleop
    def teleopInit(self):
        self.arm_controller.setToggleButtons()
        self.drive_train.reset_slew()
        logging.info("Entering Teleop")
        global frc_stage
        frc_stage = "TELEOP"

    def teleopPeriodic(self):
        if self.profile:
            self.drive_train.swerveDrive(self.joystick, profile=self.profile)
        else:
            self.drive_train.swerveDrive(self.joystick)
        self.arm_controller.setArm(self.joystick)
        global fms_attached
        fms_attached = wpilib.DriverStation.isFMSAttached()
        if self.use_threading:
            self.manageThreads()
        else:
            self.doActions()
        
    def manageThreads(self):
        # Check all threads and make sure they are alive
        for thread in self.threads:
            if thread["thread"].is_alive() == False:
                logging.warning(f"Thread {thread['name']} is not alive, restarting...")
                thread["thread"] = startThread(thread["name"])
                
    def doActions(self):
        encoderAction(self.encoder_publisher)
        stageBroadcasterAction(self.stage_publisher)
        dashboardAction(self.dashboard_publisher)
        dashboardSubAction(self.dashboard_subscriber)
        
    def stopThreads(self):
        global stop_threads
        stop_threads = True
        for thread in self.threads:
            thread.join()
        logging.info('All Threads Stopped')
        

if __name__ == '__main__':
    wpilib.run(Robot)