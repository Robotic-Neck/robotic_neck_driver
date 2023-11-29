import rclpy
import numpy as np

from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class IMUDriver(Node):

    def __init__(self):
        super().__init__('imu_driver')
        self.ser = serial.Serial('/dev/ttyUSB0', 57600)
        self.set_parameters()
        self.set_connections()
        self.run()
    
    def set_parameters(self):
        self.roll = 0.0
        self.pitch = 0.0 
        self.interpolation = 0.84   
    
    def set_connections(self):
        self.pitch_pub = self.create_publisher(Float32, '/rpip/pitch', 1)
        self.roll_pub = self.create_publisher(Float32, '/rpip/roll', 1)
    
    def publish_angles(self):
        if self.ser.in_waiting > 0:
            data = self.ser.readline().decode('utf-8').rstrip().split('/')
            self.roll = np.deg2rad(float(data[0]) * self.interpolation)
            self.pitch = np.deg2rad(float(data[1]) * self.interpolation)

        #print(f"roll: {self.roll} pitch: {self.pitch}")
        self.pitch_pub.publish(Float32(data=self.pitch))
        self.roll_pub.publish(Float32(data=self.roll))
    
    def run(self):
        while True:
            self.publish_angles()

def main():
    rclpy.init()
    node = IMUDriver()
    rclpy.spin(node)
    rclpy.shutdown()