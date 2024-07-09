#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from math import pi

class DifferentialDriver(Node):
    def __init__(self):
        super().__init__('cmdvel_listener')
        self.create_subscription(Twist, '/cmd_vel', self.callback, 10)
        
        self.left_pwm_pub = self.create_publisher(Int32, 'left_pwm', 10)
        self.right_pwm_pub = self.create_publisher(Int32, 'right_pwm', 10)

        self.left_pwm = Int32()
        self.right_pwm = Int32()

        self.params_setup()
        self.wheel_radius = self.wheel_diameter / 2
        self.circumference = 2 * pi * self.wheel_diameter
        self.max_speed = (self.max_rpm * self.circumference) / 60

    def params_setup(self):
        self.max_rpm = "max_rpm from params file"
        self.wheel_diameter = "wheel_diameter from params file"
        self.wheel_diameter = self.wheel_diameter / 100
        self.wheel_seperation = "wheel_seperation from params file"
        self.wheel_seperation = self.wheel_seperation / 100
        self.max_pwm_val = "twist max pwm"
        self.min_pwm_val = "twist min pwm"

    def stop(self):
        pass
    
    def get_pwm(self, left_speed, right_speed):
        
        lspeedPWM = max(min((left_speed/self.max_speed)*self.max_pwm_val, self.max_pwm_val), self.min_pwm_val)
        rspeedPWM = max(min((right_speed/self.max_speed)*self.max_pwm_val,self.max_pwm_val), self.min_pwm_val)

        return lspeedPWM, rspeedPWM
    
    def callback(self, data):  

        linear_vel = data.linear.x                                              # Linear Velocity of Robot
        angular_vel = data.angular.z                                            # Angular Velocity of Robot

        right_vel = linear_vel + (angular_vel * self.wheel_seperation) / 2      # right wheel velocity
        left_vel  = linear_vel - (angular_vel * self.wheel_seperation) / 2      # left wheel velocity
        
        left_pwm_data , right_pwm_data = self.get_pwm(left_vel, right_vel)

        #print(left_pwm_data) 
        #print(right_pwm_data) 


        self.left_pwm.data = int(left_pwm_data)
        self.right_pwm.data = int(right_pwm_data)


        self.left_pwm_pub.publish(self.left_pwm)
        self.right_pwm_pub.publish(self.right_pwm)

if __name__ == "__main__":
    rclpy.init()
    node = DifferentialDriver()
    rclpy.spin(node)
    rclpy.shutdown()