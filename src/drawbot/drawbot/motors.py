import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time
import math
import zmq
import numpy as np

from .common import euler_to_quaternion, forward_kinematics, ENCODER_PPR

class WheelEncoder(Node):
    def __init__(self):
        super().__init__('wheel_encoder')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('odom_frame', 'odom'),
                ('odom_topic', 'wheel/odometry'),
                ('base_frame', 'base_footprint'),
            ]
        )

        self.odompub = self.create_publisher(Odometry, self.get_parameter('odom_topic').value, 1)

        self.position = np.array([0, 0])
        self.velocity = np.array([0, 0])
        self.theta = 0
        self.angular_velocity = 0
        self.last_update_time = time.time()
        self.encoders = np.array([0, 0, 0, 0])
        self.wheel_vel = np.array([0, 0, 0, 0])

    def update_reading(self, new_encoders):
        now = time.time()
        delta_t = now - self.last_update_time
        self.last_update_time = now

        encoder_diff = new_encoders - self.encoders
        self.encoders = new_encoders
        for idx, encoder in enumerate(encoder_diff):
            self.wheel_vel[idx] = ((2*math.pi * encoder)/ENCODER_PPR)/delta_t
        relative_pos, dtheta = forward_kinematics(self.wheel_vel*delta_t)
        self.velocity = (relative_pos / delta_t)*0.1 + self.velocity*0.9
        self.theta += dtheta
        self.angular_velocity = dtheta/delta_t
        x = relative_pos[0] * math.cos(self.theta) + self.position[0] + relative_pos[1] * math.sin(self.theta)
        y = relative_pos[0] * math.sin(self.theta) + self.position[1] + relative_pos[1] * math.cos(self.theta)
        self.position = np.array([x, y])

    def publish_reading(self):
        msg = Odometry()
        msg.header.frame_id = self.get_parameter('odom_frame').value
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.child_frame_id = self.get_parameter('base_frame').value
        msg.pose.pose.position.x = self.position[0]
        msg.pose.pose.position.y = self.position[1]
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = euler_to_quaternion(0.0, 0.0, self.theta)
        msg.twist.twist.linear.x = self.velocity[0]
        msg.twist.twist.linear.y = self.velocity[1]
        msg.twist.twist.angular.z = self.angular_velocity

        msg.pose.covariance[0] = 0.1
        msg.pose.covariance[7] = 0.1
        msg.pose.covariance[14] = 0.1
        msg.pose.covariance[21] = 0.1
        msg.pose.covariance[28] = 0.1
        msg.pose.covariance[35] = 0.1

        msg.twist.covariance[0] = 0.1
        msg.twist.covariance[7] = 0.1
        msg.twist.covariance[14] = 0.1
        msg.twist.covariance[21] = 0.1
        msg.twist.covariance[28] = 0.1
        msg.twist.covariance[35] = 0.1

        self.odompub.publish(msg)

def main(args=None):
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://localhost:5560")
    socket.setsockopt_string(zmq.SUBSCRIBE, "WHEEL_ENCODER")

    rclpy.init(args=args)
    encoder = WheelEncoder()
    
    while rclpy.ok():
        cmd = socket.recv().decode('UTF-8')
        if cmd.startswith("WHEEL_ENCODER"):
            topic, fr, rl, fl, rr = cmd.split(" ")
            encoders = np.array([int(fr), int(rl), int(fl), int(rr)])
            encoder.update_reading(encoders)
            encoder.publish_reading()

    encoder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()