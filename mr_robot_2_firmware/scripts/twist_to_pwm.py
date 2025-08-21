#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from math import pi

class DifferentialDriver(Node):
    def __init__(self):
        super().__init__('cmdvel_listener')
        
        # Parameters for testing
        self.motor_rpm = 100  # Max RPM of your motor
        self.wheel_diameter = 0.394  # Wheel diameter in meters
        self.wheel_separation = 0.24  # Distance between the wheels in meters
        self.max_pwm_val = 255  # Max PWM value
        self.min_pwm_val = -255  # Min PWM value

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback,
            10
        )

        self.pwm_pub = self.create_publisher(Int32, '/pwm', 10)

        self.pwm = Int32()
        # self.pwm = [0,0]
    

        self.wheel_radius = self.wheel_diameter / 2
        self.circumference = pi * self.wheel_diameter
        self.max_speed = (self.circumference * self.motor_rpm) / 60

        self.get_logger().info('Differential Drive Initialized with following Params-')
        self.get_logger().info(f'Motor Max RPM: {self.motor_rpm} RPM')
        self.get_logger().info(f'Wheel Diameter: {self.wheel_diameter} m')
        self.get_logger().info(f'Wheel Separation: {self.wheel_separation} m')
        self.get_logger().info(f'Robot Max Speed: {self.max_speed} m/sec')
        self.get_logger().info(f'Robot Max PWM: {self.max_pwm_val}')

    def stop(self):
        self.pwm.data = 0
        self.pwm_pub.publish(self.pwm)

    def get_pwm(self, left_speed, right_speed):
        lspeedPWM = int(max(min((left_speed / self.max_speed) * self.max_pwm_val, self.max_pwm_val), self.min_pwm_val))
        rspeedPWM = int(max(min((right_speed / self.max_speed) * self.max_pwm_val, self.max_pwm_val), self.min_pwm_val))

        return lspeedPWM, rspeedPWM
    
    def callback(self, data):
        linear_vel = data.linear.x  # Linear Velocity of Robot
        angular_vel = data.angular.z  # Angular Velocity of Robot

        right_vel = linear_vel + (angular_vel * self.wheel_separation) / 2  # right wheel velocity
        left_vel = linear_vel - (angular_vel * self.wheel_separation) / 2  # left wheel velocity
        
        left_pwm_data, right_pwm_data = self.get_pwm(left_vel, right_vel)

        left_pwm_data = max(-255, min(255, left_pwm_data))
        right_pwm_data = max(-255, min(255, right_pwm_data))

        self.pwm.data = (left_pwm_data+255*2)*1000+(right_pwm_data+255*2)
        print(f"{self.pwm.data//1000 - 510},{self.pwm.data%1000 - 510} ")
        # print(self.pwm[1][1])
    
        self.pwm_pub.publish(self.pwm)

def main(args=None):
    rclpy.init(args=args)
    differential_driver = DifferentialDriver()

    rclpy.spin(differential_driver)

    differential_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
