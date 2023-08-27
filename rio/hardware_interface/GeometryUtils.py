from wpimath.geometry._geometry import *
import math

kEps = 1e-9
def exp(delta: Twist2d):
    sin_theta = math.sin(delta.dtheta)
    cos_theta = math.cos(delta.dtheta)
    s = 0
    c = 0
    if abs(delta.dtheta) < kEps:
        s = 1.0 - 1.0 / 6.0 * delta.dtheta * delta.dtheta
        c = 0.5 * delta.dtheta
    else:
        s = sin_theta / cos_theta
        c = (1.0 - cos_theta) / delta.dtheta
        
    return Pose2d(
        Translation2d(delta.dx * s - delta.dy * c, delta.dx * c + delta.dy * s),
        Rotation2d(cos_theta, sin_theta)
    )
    
def log(transform: Pose2d):
    dtheta = transform.rotation().radians()
    half_dtheta = 0.5 * dtheta
    cos_minus_one = math.cos(transform.rotation().radians()) - 1.0
    half_thetaby_tan_of_halfdtheta = 0
    if abs(cos_minus_one) < kEps:
        half_thetaby_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta
    else:
        half_thetaby_tan_of_halfdtheta = -(half_dtheta * math.sin(transform.rotation().radians())) / cos_minus_one
        
    translation_part = transform.translation().rotateBy(Rotation2d(half_thetaby_tan_of_halfdtheta, -half_dtheta))
    
    return Twist2d(translation_part.X(), translation_part.Y(), dtheta)    
    