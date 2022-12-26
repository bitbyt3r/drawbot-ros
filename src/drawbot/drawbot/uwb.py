import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import PoseWithCovarianceStamped

import serial
import time
import os

class UWB(Node):
    def __init__(self):
        print("Initializing")
        super().__init__('uwb')
        self.declare_parameter("port_name", "/dev/ttyACM0")
        self.declare_parameter("refresh_interval", 0.1)
        self.declare_parameter("frame", "uwb")
        self.declare_parameter("topic", "uwb")

        print("Opening ports")
        self.port = serial.Serial(self.get_parameter('port_name').value, 115200, timeout=1)
        self.initialized = False

        self.port_name = self.get_parameter("port_name").value
        self.timer = self.create_timer(self.get_parameter("refresh_interval").value, self.update_position)
        self.posepub = self.create_publisher(PoseWithCovarianceStamped, self.get_parameter('topic').value, 1)
        self.initialize()

    def shutdown(self):
        self.port.close()

    def parameters_callback(self, params):
        success = True
        for param in params:
            if param.name == "port_name":
                if not os.path.exists(param.value):
                    success = False
                    continue
                if self.port:
                    self.port.close()
                try:
                    self.port = serial.Serial(param.value, 115200, timeout=0)
                    self.port_name = param.value
                    self.initialize()
                except:
                    success = False
                    self.port = serial.Serial(self.port_name, 115200, timeout=0)
            
            if param.name == "refresh_interval":
                self.timer.cancel()
                self.timer = self.create_timer(self.get_parameter("refresh_interval").value, self.update_position)

        return SetParametersResult(successful=success)

    def initialize(self):
        self.get_logger().info(f"Initializing UWB")
        self.port.write(b"\r\r")
        time.sleep(1)
        self.port.write(b"reset\r")
        time.sleep(1)
        self.port.write(b"\r\r")
        time.sleep(1)
        self.port.write(b"lep\r")
        time.sleep(0.5)
        self.port.reset_input_buffer()
        self.initialized = True

    def update_position(self):
        if not self.port:
            raise RuntimeError("You must instantiate this class as a contextmanager before calling update_position")
        if not self.initialized:
            return
        data = self.port.read_until(expected=b"\r\n").decode('ASCII').strip()
        try:
            x, y, z, qf = list(map(float, data.split(",")[1:]))
            
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = self.get_parameter("frame").value
            msg.header.stamp = self.get_clock().now().to_msg()
            
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.position.z = z

            # qf is the measurement quality on a scale 0-100. Assume worst case is 40cm and best case is 20cm
            covariance = (0.4 - (qf/100)*0.2)**2
            msg.pose.covariance[0] = covariance
            msg.pose.covariance[7] = covariance
            msg.pose.covariance[14] = covariance

            self.posepub.publish(msg)
        except (ValueError, IndexError):
            self.get_logger().info(f"Could not parse position from {data}")
            return

def main(args=None):
    rclpy.init(args=args)
    uwb = UWB()
    try:
        rclpy.spin(uwb)
        rclpy.shutdown(uwb)
    except:
        uwb.shutdown()

if __name__ == '__main__':
    main()