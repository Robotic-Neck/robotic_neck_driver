# https://index.ros.org/p/joy

import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16

class RoboticNeckJoy(Node):

    def __init__(self):
        super().__init__('robotic_neck_joy')
        self.set_parameters()
        self.set_connections()
    
    def set_parameters(self):
        self.max_value = 255
        self.right_value = Int16(data=0)
        self.left_value = Int16(data=0)
    
    def set_connections(self):
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 1)
        self.right_motor_pub = self.create_publisher(Int16, '/rpip/motor_right_sub', 1)
        self.left_motor_pub = self.create_publisher(Int16, '/rpip/motor_left_sub', 1)

    def joy_callback(self, msg):
        
        if msg.buttons[4] == 1:
            self.right_value.data = int(self.max_value * msg.axes[1])
            self.left_value.data = int(-1 * self.max_value * msg.axes[4])
        
        else:
            self.right_value.data = 0
            self.left_value.data = 0

        self.right_motor_pub.publish(self.right_value)
        self.left_motor_pub.publish(self.left_value)
   

def main():
    rclpy.init()
    node = RoboticNeckJoy()
    rclpy.spin(node)
    rclpy.shutdown()