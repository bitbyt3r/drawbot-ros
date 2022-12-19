from geometry_msgs.msg import Quaternion
import numpy as np

WHEEL_DISTANCE = sum([0.049, 0.1175])
ENCODER_PPR = 450
WHEEL_RADIUS = 0.040

def inverse_kinematics(move):
    x, y, theta = move
    fr = (y - x + WHEEL_DISTANCE*theta) / WHEEL_RADIUS
    fl = (y + x - WHEEL_DISTANCE*theta) / WHEEL_RADIUS
    rl = (y - x - WHEEL_DISTANCE*theta) / WHEEL_RADIUS
    rr = (y + x + WHEEL_DISTANCE*theta) / WHEEL_RADIUS
    return np.array([fr, fl, rl, rr])

def forward_kinematics(wheel_rotations):
    fr, fl, rl, rr = wheel_rotations
    y = sum([fr, fl, rl, rr]) * WHEEL_RADIUS / 4
    x = ((fl + rr) - (fr + rl)) * WHEEL_RADIUS / 4
    theta = ((fr + rr) - (fl + rl)) * WHEEL_RADIUS / (4 * WHEEL_DISTANCE)
    return np.array([x, y, theta])

def  euler_to_quaternion(self, roll, pitch, yaw):
    q = Quaternion()
    q.x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    q.y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    q.z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    q.w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return q
