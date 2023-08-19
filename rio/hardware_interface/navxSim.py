import math
from squaternion import Quaternion

from wpimath.geometry import Rotation2d
class NavxSim:
    def __init__(self) -> None:
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.angular_velocity_x = 0
        self.angular_velocity_y = 0
        self.angular_velocity_z = 0
        self.linear_acceleration_x = 0
        self.linear_acceleration_y = 0
        self.linear_acceleration_z = 0
        
        self.offset = 0
        
        self.quat = Quaternion()

    def update(self, w, x, y, z, angular_velocity_x, angular_velocity_y, angular_velocity_z, linear_acceleration_x, linear_acceleration_y, linear_acceleration_z):
        self.quat = Quaternion(w, x, y, z)
        euler = self.quat.to_euler()
        self.roll = euler[0]
        self.pitch = euler[1]
        self.yaw = euler[2]
        self.angular_velocity_x = angular_velocity_x
        self.angular_velocity_y = angular_velocity_y
        self.angular_velocity_z = angular_velocity_z
        self.linear_acceleration_x = linear_acceleration_x
        self.linear_acceleration_y = linear_acceleration_y
        self.linear_acceleration_z = linear_acceleration_z
        
    def zeroYaw(self):
        self.offset = self.yaw

    def getYaw(self):
        return self.yaw - self.offset
    
    def getYawDegrees(self):
        return math.degrees(self.getYaw())

    def getPitch(self):
        return self.pitch
    
    def getPitchDegrees(self):
        return math.degrees(self.pitch)

    def getRoll(self):
        return self.roll
    
    def getRollDegrees(self):
        return math.degrees(self.roll)
    
    def getRotation2d(self):
        return Rotation2d.fromDegrees(self.getYawDegrees())
    
    def getQuaternionWXYZ(self):
        return [self.quat.w, self.quat.x, self.quat.y, self.quat.z]
    
    def getAngularVelocityXYZ(self):
        return [self.angular_velocity_x, self.angular_velocity_y, self.angular_velocity_z]
    
    def getLinearAccelerationXYZ(self):
        return [self.linear_acceleration_x, self.linear_acceleration_y, self.linear_acceleration_z]