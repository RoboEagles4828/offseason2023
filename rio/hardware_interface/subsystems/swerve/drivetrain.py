import wpilib
from wpimath.geometry._geometry import *
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState
from hardware_interface.motion_magic import MotionMagic
from hardware_interface import GeometryUtils
import ctre.sensors
import math
import time
import logging
from wpimath.filter import SlewRateLimiter
from hardware_interface.joystick import Joystick
import navx
from hardware_interface.toggle import ToggleButton
from hardware_interface.navxSim import NavxSim
from hardware_interface.subsystems.swerve.swerve_constants import *
from hardware_interface.subsystems.swerve.swerve_module import SwerveModule

class DriveTrain():
    def __init__(self):
        self.last_cmds = { "name" : getJointList(), "position": [0.0]*len(getJointList()), "velocity": [0.0]*len(getJointList()) }
        self.last_cmds_time = time.time()
        self.warn_timeout = True
        self.front_left = SwerveModule(MODULE_CONFIG["front_left"])
        self.front_right = SwerveModule(MODULE_CONFIG["front_right"])
        self.rear_left = SwerveModule(MODULE_CONFIG["rear_left"])
        self.rear_right = SwerveModule(MODULE_CONFIG["rear_right"])
        self.front_left_location = MODULE_CONFIG["front_left"]["location"]
        self.front_right_location = MODULE_CONFIG["front_right"]["location"]
        self.rear_left_location = MODULE_CONFIG["rear_left"]["location"]
        self.rear_right_location = MODULE_CONFIG["rear_right"]["location"]
        self.kinematics = SwerveDrive4Kinematics(self.front_left_location, self.front_right_location, self.rear_left_location, self.rear_right_location)
        self.navx = navx.AHRS.create_spi()
        self.navx_sim = NavxSim()
        self.navx.calibrate()
        self.speeds = ChassisSpeeds(0, 0, 0)
        self.wheel_radius = 0.0508

        self.ROBOT_MAX_TRANSLATIONAL = 10.0 #16.4041995 # 5.0 # m/s
        self.ROBOT_MAX_ROTATIONAL = 10.0 #16.4041995 * math.pi #rad/s

        self.MODULE_MAX_SPEED = 10.0 #16.4041995 # m/s

        self.move_scale_x = self.ROBOT_MAX_TRANSLATIONAL
        self.move_scale_y = self.ROBOT_MAX_TRANSLATIONAL
        self.turn_scale = self.ROBOT_MAX_ROTATIONAL

        self.slew_X = SlewRateLimiter(2)
        self.slew_Y = SlewRateLimiter(2)
        self.slew_Z = SlewRateLimiter(2)
        self.slew_slow_translation = SlewRateLimiter(0.5)
        self.slew_slow_rotation = SlewRateLimiter(0.5)

        self.slew_X.reset(0)
        self.slew_Y.reset(0)
        self.slew_Z.reset(0)

        self.field_oriented_button = ToggleButton(7, False)
        self.field_oriented_value = False

        self.auto_turn_value = "off"

        self.profile_selector = wpilib.SendableChooser()
        self.profile_selector.setDefaultOption("Competition", (5.0, 5.0))
        self.profile_selector.addOption("Workshop", (2.5, 1.25))

        self.whine_remove_selector = wpilib.SendableChooser()
        self.whine_remove_selector.setDefaultOption("OFF", False)
        self.whine_remove_selector.addOption("ON", True)
        
        self.angle_source_selector = wpilib.SendableChooser()
        self.angle_source_selector.addOption("Angle", "angle")
        self.angle_source_selector.addOption("Yaw", "yaw")
        self.angle_source_selector.setDefaultOption("Normal", "normal")
        
        self.module_state = [SwerveModuleState(0, Rotation2d(0))]*4
        

        self.slow = False

        self.module_lookup = \
        {
            'front_left_axle_joint': self.front_left,
            'front_right_axle_joint': self.front_right,
            'rear_left_axle_joint': self.rear_left,
            'rear_right_axle_joint': self.rear_right,
        }
        self.last_print = ""

        self.last_state = ChassisSpeeds(0, 0, 0)

        self.motor_temps = []
        
        self.linX = 0
        self.linY = 0
        self.angZ = 0
        
        self.navx_offset = 0
        
        self.motor_vels = []
        self.motor_pos = []
        
        real_mm_accel = (8.0 - 2.0) / accelerationConstant / velocityCoefficient
        real_mm_vel = 2.0 / velocityConstant / velocityCoefficient
        
        self.new_motion_magic_1 = MotionMagic(getAxleRadians(real_mm_accel, "velocity") * 10, getAxleRadians(real_mm_vel, "velocity"))
        self.new_motion_magic_2 = MotionMagic(getAxleRadians(real_mm_accel, "velocity") * 10, getAxleRadians(real_mm_vel, "velocity"))
        self.new_motion_magic_3 = MotionMagic(getAxleRadians(real_mm_accel, "velocity") * 10, getAxleRadians(real_mm_vel, "velocity"))
        self.new_motion_magic_4 = MotionMagic(getAxleRadians(real_mm_accel, "velocity") * 10, getAxleRadians(real_mm_vel, "velocity"))
        
        self.front_left_state = SwerveModuleState(0, Rotation2d(0))
        self.front_right_state = SwerveModuleState(0, Rotation2d(0))
        self.rear_left_state = SwerveModuleState(0, Rotation2d(0))
        self.rear_right_state = SwerveModuleState(0, Rotation2d(0))
        
        self.is_sim = False
        self.locked = False
        
    def reset_slew(self):
        self.slew_X.reset(0)
        self.slew_Y.reset(0)
        self.slew_Z.reset(0)

    def getEncoderData(self):
        names = [""]*8
        positions = [0]*8
        velocities = [0]*8
        efforts = [0]*8

        encoderInfo = []
        encoderInfo += self.front_left.getEncoderData() 
        encoderInfo += self.front_right.getEncoderData()
        encoderInfo += self.rear_left.getEncoderData()
        encoderInfo += self.rear_right.getEncoderData()
        assert len(encoderInfo) == 8
        for index, encoder in enumerate(encoderInfo):
            names[index] = encoder['name']
            positions[index] = encoder['position']
            velocities[index] = encoder['velocity']
            efforts[index] = encoder['effort']
        return { "name": names, "position": positions, "velocity": velocities, "effort": efforts }

    def stop(self):
        self.front_left.neutralize_module()
        self.front_right.neutralize_module()
        self.rear_left.neutralize_module()
        self.rear_right.neutralize_module()

    def testModule(self):
        self.rear_right.encoderOffsetTest()

    def allZero(self, speeds: ChassisSpeeds):
        if 0 <= abs(speeds.vx) <= 0.1:
            if 0 <= abs(speeds.vy) <= 0.1:
                if 0 <= abs(speeds.omega) <= 0.1:
                    return True
        return False
    
    def customOptimize(self, desiredState: SwerveModuleState, currentAngle: Rotation2d):
        delta = desiredState.angle.__sub__(currentAngle)
        if abs(delta.degrees()) > 90:
            return SwerveModuleState(desiredState.speed * -1.0, desiredState.angle.rotateBy(Rotation2d.fromDegrees(180)))
        else:
            return SwerveModuleState(desiredState.speed, desiredState.angle)
        
    def correctForDynamics(self, originalSpeeds: ChassisSpeeds):
        LOOP_TIME_S = 0.02
        futureRobotPose = Pose2d(
            originalSpeeds.vx * LOOP_TIME_S,
            originalSpeeds.vy * LOOP_TIME_S,
            Rotation2d(originalSpeeds.omega * LOOP_TIME_S)
        )
        twistForPose = GeometryUtils.log(futureRobotPose)
        updatedSpeeds = ChassisSpeeds(
            twistForPose.dx / LOOP_TIME_S,
            twistForPose.dy / LOOP_TIME_S,
            twistForPose.dtheta / LOOP_TIME_S
        )
        return updatedSpeeds

    def swerveDrive(self, joystick: Joystick):
        print(f"SECOND_ORDER: {ENABLE_2ND_ORDER}")
        angle_source = self.angle_source_selector.getSelected()
        # self.ROBOT_MAX_TRANSLATIONAL = self.profile_selector.getSelected()[0]
        # self.ROBOT_MAX_ROTATIONAL = self.profile_selector.getSelected()[1]
        # self.MODULE_MAX_SPEED = self.profile_selector.getSelected()[0]

        # slew 
        # gives joystick ramping
        # linearX = self.slew_X.calculate(math.pow(joystick.getData()["axes"][1], 5)) * self.ROBOT_MAX_TRANSLATIONAL / self.move_scale_x
        # linearY = self.slew_Y.calculate(math.pow(joystick.getData()["axes"][0], 5)) * -self.ROBOT_MAX_TRANSLATIONAL / self.move_scale_y
        # angularZ = self.slew_Z.calculate(math.pow(joystick.getData()["axes"][3], 5)) * self.ROBOT_MAX_ROTATIONAL / self.turn_scale
        
        linearX = math.pow(joystick.getData()["axes"][1], 5) * self.ROBOT_MAX_TRANSLATIONAL / self.move_scale_x
        linearY = math.pow(joystick.getData()["axes"][0], 5) * -self.ROBOT_MAX_TRANSLATIONAL / self.move_scale_y
        angularZ = math.pow(joystick.getData()["axes"][3], 5) * self.ROBOT_MAX_ROTATIONAL / self.turn_scale

        self.linX = linearX
        self.linY = linearY
        self.angZ = angularZ

        if joystick.getData()["buttons"][7] == 1.0:
            self.field_oriented_value = not self.field_oriented_button.toggle(joystick.getData()["buttons"])
        else:
            self.field_oriented_value = not self.field_oriented_button.toggle(joystick.getData()["buttons"])

        if joystick.getData()["buttons"][6] == 1.0:
            self.navx.reset()
            self.navx_offset = 0
            if self.is_sim:
                self.navx_sim.zeroYaw()

        if joystick.getData()["axes"][5] == 1.0:
            self.slow = True
            self.move_scale_x = 1.0
            self.move_scale_y = 1.0
            self.turn_scale = 1.0
        else:            
            self.slow = False
            self.move_scale_x = 2.0
            self.move_scale_y = 2.0
            self.turn_scale = 2.0

        if joystick.getData()["axes"][6] == 1.0:
            self.auto_turn_value = "load"
        elif joystick.getData()["axes"][6] == -1.0:
            self.auto_turn_value = "score"
        else:
            self.auto_turn_value = "off"

        if self.field_oriented_value and self.auto_turn_value == "off":            
            # field  oriented
            navx_value = Rotation2d()
            if angle_source == "angle":
                navx_value = Rotation2d.fromDegrees(self.navx.getAngle()).__mul__(-1)
            elif angle_source == "yaw":
                navx_value = Rotation2d.fromDegrees(self.navx.getYaw()).__mul__(-1)
            elif angle_source == "normal":
                navx_value = Rotation2d.fromDegrees(self.navx.getRotation2d().__mul__(-1).degrees() + self.navx_offset)
                
            if ENABLE_2ND_ORDER:
                self.speeds = self.correctForDynamics(ChassisSpeeds.fromFieldRelativeSpeeds(linearX, linearY, angularZ, navx_value))
            else:
                self.speeds = ChassisSpeeds.fromFieldRelativeSpeeds(linearX, linearY, angularZ, navx_value)
                
            print(f"X: {self.speeds.vx} Y: {self.speeds.vy} Z: {self.speeds.omega}")
        else:
            if ENABLE_2ND_ORDER:
                self.speeds = self.correctForDynamics(ChassisSpeeds(linearX, linearY, angularZ))
            else:    
                self.speeds = ChassisSpeeds(linearX, linearY, angularZ)
            #if self.last_print != f"linX: {round(self.speeds.vx, 2)} linY: {round(self.speeds.vy, 2)} angZ: {round(self.speeds.omega, 2)} MoveScaleX: {round(self.move_scale_x, 2)} MoveScaleY: {round(self.move_scale_y, 2)} TurnScale: {round(self.turn_scale, 2)}":
            #logging.info(f"linX: {round(self.speeds.vx, 2)} linY: {round(self.speeds.vy, 2)} angZ: {round(self.speeds.omega, 2)} MoveScaleX: {round(self.move_scale_x, 2)} MoveScaleY: {round(self.move_scale_y, 2)} TurnScale: {round(self.turn_scale, 2)}")
                #self.last_print = f"linX: {round(self.speeds.vx, 2)} linY: {round(self.speeds.vy, 2)} angZ: {round(self.speeds.omega, 2)} MoveScaleX: {round(self.move_scale_x, 2)} MoveScaleY: {round(self.move_scale_y, 2)} TurnScale: {round(self.turn_scale, 2)}"

        self.module_state = self.kinematics.toSwerveModuleStates(self.speeds)
        
        # normalize speeds
        # if the speeds are greater than the max speed, scale them down
        self.kinematics.desaturateWheelSpeeds(self.module_state, self.MODULE_MAX_SPEED)

        self.front_left_state: SwerveModuleState = self.module_state[0]
        self.front_right_state: SwerveModuleState = self.module_state[1]
        self.rear_left_state: SwerveModuleState = self.module_state[2]
        self.rear_right_state: SwerveModuleState = self.module_state[3]

        # optimize states
        #This makes the modules take the shortest path to the desired angle
        self.front_left_state = SwerveModuleState.optimize(self.front_left_state, Rotation2d(self.front_left.getEncoderPosition()))
        self.front_right_state = SwerveModuleState.optimize(self.front_right_state, Rotation2d(self.front_right.getEncoderPosition()))
        self.rear_left_state = SwerveModuleState.optimize(self.rear_left_state, Rotation2d(self.rear_left.getEncoderPosition()))
        self.rear_right_state = SwerveModuleState.optimize(self.rear_right_state, Rotation2d(self.rear_right.getEncoderPosition()))
        
        # using custom optimize
        # self.front_left_state = self.customOptimize(self.front_left_state, Rotation2d(self.front_left.getEncoderPosition()))
        # self.front_right_state = self.customOptimize(self.front_right_state, Rotation2d(self.front_right.getEncoderPosition()))
        # self.rear_left_state = self.customOptimize(self.rear_left_state, Rotation2d(self.rear_left.getEncoderPosition()))
        # self.rear_right_state = self.customOptimize(self.rear_right_state, Rotation2d(self.rear_right.getEncoderPosition()))

        self.front_left.set(self.front_left_state)
        self.front_right.set(self.front_right_state)
        self.rear_left.set(self.rear_left_state)
        self.rear_right.set(self.rear_right_state)
        
        

        #logging.info(f"angz: {angularZ}, FL: {self.front_left_state.speed}, FR: {self.front_right_state.speed}, BL: {self.rear_left_state.speed}, BR: {self.rear_right_state.speed}")

        self.motor_vels = [radiansToMeters(getWheelRadians(self.front_left.wheel_motor.getSelectedSensorVelocity(), "velocity")), radiansToMeters(getWheelRadians(self.front_right.wheel_motor.getSelectedSensorVelocity(), "velocity")), radiansToMeters(getWheelRadians(self.rear_left.wheel_motor.getSelectedSensorVelocity(), "velocity")), radiansToMeters(getWheelRadians(self.rear_right.wheel_motor.getSelectedSensorVelocity(), "velocity"))]
        self.motor_pos = [self.front_left.getEncoderPosition(), self.front_right.getEncoderPosition(), self.rear_left.getEncoderPosition(), self.rear_right.getEncoderPosition()]
        self.motor_temps = [self.front_left.wheel_motor.getTemperature(), self.front_right.wheel_motor.getTemperature() ,self.rear_left.wheel_motor.getTemperature(), self.rear_right.wheel_motor.getTemperature()]

        self.last_state = self.speeds
        
        if self.field_oriented_value:
            self.print = f"Navx: {Rotation2d.fromDegrees(self.navx.getRotation2d().__mul__(-1).degrees() + self.navx_offset)} "
        else:
            self.print = ""

        logging.info(f"Navx: {Rotation2d.fromDegrees(self.navx.getRotation2d().__mul__(-1).degrees() + self.navx_offset)}")
        #logging.info(f"FR: {self.front_left_state.speed}, {self.front_left_state.angle.radians()} | Current Angle: {self.front_left.getEncoderPosition()}")
        # logging.info(f"{self.print}linX: {round(self.speeds.vx, 2)} linY: {round(self.speeds.vy, 2)} angZ: {round(self.speeds.omega, 2)} FL: {round(radiansToMeters(getWheelRadians(self.front_left.wheel_motor.getSelectedSensorVelocity(), 'velocity')), 2)}")
        
    def getModuleCommand(self):
        data = dict()

        if self.is_sim:
            if ENABLE_2ND_ORDER:
                self.speeds = self.correctForDynamics(ChassisSpeeds.fromFieldRelativeSpeeds(self.linX, self.linY, self.angZ, Rotation2d.fromDegrees(self.navx_sim.getRotation2d().__mul__(-1).degrees() + self.navx_offset)))
            else:
                self.speeds = ChassisSpeeds.fromFieldRelativeSpeeds(self.linX, self.linY, self.angZ, Rotation2d.fromDegrees(self.navx_sim.getRotation2d().__mul__(-1).degrees() + self.navx_offset))
            #print(self.navx_sim.getRotation2d())
            
            self.module_state = self.kinematics.toSwerveModuleStates(self.speeds)
            
            # normalize speeds
            # if the speeds are greater than the max speed, scale them down
            self.kinematics.desaturateWheelSpeeds(self.module_state, self.speeds, self.MODULE_MAX_SPEED, self.ROBOT_MAX_TRANSLATIONAL, self.ROBOT_MAX_ROTATIONAL)
            
            if not self.locked:

                self.front_left_state: SwerveModuleState = self.module_state[0]
                self.front_right_state: SwerveModuleState = self.module_state[1]
                self.rear_left_state: SwerveModuleState = self.module_state[2]
                self.rear_right_state: SwerveModuleState = self.module_state[3]

                # optimize states
                #This makes the modules take the shortest path to the desired angle
                self.front_left_state = SwerveModuleState.optimize(self.front_left_state, Rotation2d(self.front_left.getEncoderPosition()))
                self.front_right_state = SwerveModuleState.optimize(self.front_right_state, Rotation2d(self.front_right.getEncoderPosition()))
                self.rear_left_state = SwerveModuleState.optimize(self.rear_left_state, Rotation2d(self.rear_left.getEncoderPosition()))
                self.rear_right_state = SwerveModuleState.optimize(self.rear_right_state, Rotation2d(self.rear_right.getEncoderPosition()))            
        
        m1_val = self.new_motion_magic_1.getNextVelocity(self.front_left_state.angle.radians(), self.front_left.getMotorPosition())
        m2_val = self.new_motion_magic_2.getNextVelocity(self.front_right_state.angle.radians(), self.front_right.getMotorPosition())
        m3_val = self.new_motion_magic_3.getNextVelocity(self.rear_left_state.angle.radians(), self.rear_left.getMotorPosition())
        m4_val = self.new_motion_magic_4.getNextVelocity(self.rear_right_state.angle.radians(), self.rear_right.getMotorPosition())
        data["name"] = getJointList()
        data["velocity"] = [
            metersToRadians(self.front_left_state.speed) * SCALING_FACTOR_FIX,
            m1_val * SCALING_FACTOR_FIX,
            metersToRadians(self.front_right_state.speed) * SCALING_FACTOR_FIX,
            m2_val * SCALING_FACTOR_FIX,
            metersToRadians(self.rear_left_state.speed) * SCALING_FACTOR_FIX,
            m3_val * SCALING_FACTOR_FIX,
            metersToRadians(self.rear_right_state.speed) * SCALING_FACTOR_FIX,
            m4_val * SCALING_FACTOR_FIX,
        ]
        data["position"] = [0.0]*8
        
        #print(f"{round(self.linX, 2)} {round(self.linY, 2)} {round(self.angZ, 2)} | {round(self.front_left.getMotorPosition(), 2)} {round(self.front_left.getMotorPosition(), 2)} {round(self.front_left.getMotorPosition(), 2)} {round(self.front_left.getMotorPosition(), 2)}")
        
        return data

    def swerveDriveAuton(self, linearX, linearY, angularZ):
        ROBOT_MAX_TRANSLATIONAL = 5.0
        ROBOT_MAX_ROTATIONAL = 5.0
        MODULE_MAX_SPEED = 5.0
        if ENABLE_2ND_ORDER:
            self.speeds = self.correctForDynamics(ChassisSpeeds(linearX*ROBOT_MAX_TRANSLATIONAL, -linearY*ROBOT_MAX_TRANSLATIONAL, angularZ*ROBOT_MAX_ROTATIONAL))
        else:
            self.speeds = ChassisSpeeds(linearX*ROBOT_MAX_TRANSLATIONAL, -linearY*ROBOT_MAX_TRANSLATIONAL, angularZ*ROBOT_MAX_ROTATIONAL)
        self.linX = linearX*ROBOT_MAX_TRANSLATIONAL
        self.linY = linearY*ROBOT_MAX_TRANSLATIONAL
        self.angZ = angularZ*ROBOT_MAX_ROTATIONAL

        self.module_state = self.kinematics.toSwerveModuleStates(self.speeds)
        self.kinematics.desaturateWheelSpeeds(self.module_state, self.speeds, MODULE_MAX_SPEED, ROBOT_MAX_TRANSLATIONAL, ROBOT_MAX_ROTATIONAL)

        self.front_left_state: SwerveModuleState = self.module_state[0]
        self.front_right_state: SwerveModuleState = self.module_state[1]
        self.rear_left_state: SwerveModuleState = self.module_state[2]
        self.rear_right_state: SwerveModuleState = self.module_state[3]

        # optimize states
        self.front_left_state = SwerveModuleState.optimize(self.front_left_state, Rotation2d(self.front_left.getEncoderPosition()))
        self.front_right_state = SwerveModuleState.optimize(self.front_right_state, Rotation2d(self.front_right.getEncoderPosition()))
        self.rear_left_state = SwerveModuleState.optimize(self.rear_left_state, Rotation2d(self.rear_left.getEncoderPosition()))
        self.rear_right_state = SwerveModuleState.optimize(self.rear_right_state, Rotation2d(self.rear_right.getEncoderPosition()))
        
        # using custom optimize
        # self.front_left_state = self.customOptimize(self.front_left_state, Rotation2d(self.front_left.getEncoderPosition()))
        # self.front_right_state = self.customOptimize(self.front_right_state, Rotation2d(self.front_right.getEncoderPosition()))
        # self.rear_left_state = self.customOptimize(self.rear_left_state, Rotation2d(self.rear_left.getEncoderPosition()))
        # self.rear_right_state = self.customOptimize(self.rear_right_state, Rotation2d(self.rear_right.getEncoderPosition()))

        self.front_left.set(self.front_left_state)
        self.front_right.set(self.front_right_state)
        self.rear_left.set(self.rear_left_state)
        self.rear_right.set(self.rear_right_state)
        
    def swerveDriveAutonFieldOriented(self, linearX, linearY, angularZ):
        ROBOT_MAX_TRANSLATIONAL = 5.0
        ROBOT_MAX_ROTATIONAL = 5.0
        MODULE_MAX_SPEED = 5.0
        if ENABLE_2ND_ORDER:
            self.speeds = self.correctForDynamics(ChassisSpeeds.fromFieldRelativeSpeeds(linearX*ROBOT_MAX_TRANSLATIONAL, -linearY*ROBOT_MAX_TRANSLATIONAL, angularZ*ROBOT_MAX_ROTATIONAL, Rotation2d.fromDegrees(self.navx.getRotation2d().__mul__(-1).degrees() + self.navx_offset)))
        else:
            self.speeds = ChassisSpeeds.fromFieldRelativeSpeeds(linearX*ROBOT_MAX_TRANSLATIONAL, -linearY*ROBOT_MAX_TRANSLATIONAL, angularZ*ROBOT_MAX_ROTATIONAL, Rotation2d.fromDegrees(self.navx.getRotation2d().__mul__(-1).degrees() + self.navx_offset))
        self.linX = linearX*ROBOT_MAX_TRANSLATIONAL
        self.linY = linearY*ROBOT_MAX_TRANSLATIONAL
        self.angZ = angularZ*ROBOT_MAX_ROTATIONAL

        module_state = self.kinematics.toSwerveModuleStates(self.speeds)
        self.kinematics.desaturateWheelSpeeds(self.module_state, self.speeds, MODULE_MAX_SPEED, ROBOT_MAX_TRANSLATIONAL, ROBOT_MAX_ROTATIONAL)

        front_left_state: SwerveModuleState = module_state[0]
        front_right_state: SwerveModuleState = module_state[1]
        rear_left_state: SwerveModuleState = module_state[2]
        rear_right_state: SwerveModuleState = module_state[3]

        # optimize states
        front_left_state = SwerveModuleState.optimize(front_left_state, Rotation2d(self.front_left.getEncoderPosition()))
        front_right_state = SwerveModuleState.optimize(front_right_state, Rotation2d(self.front_right.getEncoderPosition()))
        rear_left_state = SwerveModuleState.optimize(rear_left_state, Rotation2d(self.rear_left.getEncoderPosition()))
        rear_right_state = SwerveModuleState.optimize(rear_right_state, Rotation2d(self.rear_right.getEncoderPosition()))
        
        # using custom optimize
        # self.front_left_state = self.customOptimize(self.front_left_state, Rotation2d(self.front_left.getEncoderPosition()))
        # self.front_right_state = self.customOptimize(self.front_right_state, Rotation2d(self.front_right.getEncoderPosition()))
        # self.rear_left_state = self.customOptimize(self.rear_left_state, Rotation2d(self.rear_left.getEncoderPosition()))
        # self.rear_right_state = self.customOptimize(self.rear_right_state, Rotation2d(self.rear_right.getEncoderPosition()))

        self.front_left.set(front_left_state)
        self.front_right.set(front_right_state)
        self.rear_left.set(rear_left_state)
        self.rear_right.set(rear_right_state)
        
    def lockDrive(self):
        self.locked = True
        front_left_state = SwerveModuleState(0, Rotation2d.fromDegrees(45))
        front_right_state = SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        rear_left_state = SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        rear_right_state = SwerveModuleState(0, Rotation2d.fromDegrees(45))
        
        self.front_left.set(front_left_state)
        self.front_right.set(front_right_state)
        self.rear_left.set(rear_left_state)
        self.rear_right.set(rear_right_state)
        
    def unlockDrive(self):
        self.locked = False
        front_left_state = SwerveModuleState(0, Rotation2d.fromDegrees(0))
        front_right_state = SwerveModuleState(0, Rotation2d.fromDegrees(0))
        rear_left_state = SwerveModuleState(0, Rotation2d.fromDegrees(0))
        rear_right_state = SwerveModuleState(0, Rotation2d.fromDegrees(0))
        
        self.front_left.set(front_left_state)
        self.front_right.set(front_right_state)
        self.rear_left.set(rear_left_state)
        self.rear_right.set(rear_right_state)
        
    def metersToShaftTicks(self, meters):
        wheel_circumference = 2 * math.pi * self.wheel_radius
        rotations = meters / wheel_circumference
        radians = rotations * 2 * math.pi
        ticks = getWheelShaftTicks(radians, "position")
        return ticks
    
    def shaftTicksToMeters(self, ticks):
        radians = getWheelRadians(ticks, "position")
        wheel_circumference = 2 * math.pi * self.wheel_radius
        rotations = radians / (2 * math.pi)
        meters = rotations * wheel_circumference
        return meters
    
    def set_navx_offset(self, value):
        self.navx_offset = value
        
        





