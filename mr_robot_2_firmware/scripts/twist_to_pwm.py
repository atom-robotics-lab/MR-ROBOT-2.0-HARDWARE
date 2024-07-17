#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import time
from math import pi

class DifferentialDriver(Node):
    def __init__(self):
        super().__init__('cmdvel_listener')
        
        # Parameters for testing
        self.motor_rpm = 100
        self.wheel_diameter = 0.113
        self.wheel_separation = 0.24
        self.max_pwm_val = 100
        self.min_pwm_val = -100

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback,
            10
        )

        self.left_pwm_pub = self.create_publisher(Int32, '/left_pwm', 10)
        self.right_pwm_pub = self.create_publisher(Int32, '/right_pwm', 10)

        self.left_pwm = Int32()
        self.right_pwm = Int32()  

        self.wheel_diameter = self.wheel_diameter / 100
        self.wheel_separation = self.wheel_separation / 100

        # self.params_setup()
        self.wheel_radius = self.wheel_diameter / 2
        self.circumference = 2 * pi * self.wheel_radius
        self.max_speed = (self.circumference * self.motor_rpm) / 60

        self.get_logger().info('Differential Drive Initialized with following Params-')
        self.get_logger().info(f'Motor Max RPM: {self.motor_rpm} RPM')
        self.get_logger().info(f'Wheel Diameter: {self.wheel_diameter} m')
        self.get_logger().info(f'Wheel Separation: {self.wheel_separation} m')
        self.get_logger().info(f'Robot Max Speed: {self.max_speed} m/sec')
        self.get_logger().info(f'Robot Max PWM: {self.max_pwm_val}')

    # def params_setup(self):
    #     self.motor_rpm = self.get_parameter('mr_robot_firmware.motor_rpm').get_parameter_value().integer_value
    #     self.wheel_diameter = self.get_parameter('mr_robot_firmware.wheel_diameter').get_parameter_value().double_value
    #     self.wheel_separation = self.get_parameter('mr_robot_firmware.wheel_separation').get_parameter_value().double_value
    #     self.max_pwm_val = self.get_parameter('mr_robot_firmware.twist_max_pwm').get_parameter_value().integer_value
    #     self.min_pwm_val = self.get_parameter('mr_robot_firmware.twist_min_pwm').get_parameter_value().integer_value

    def stop(self):
        self.left_pwm.data = 0
        self.right_pwm.data = 0
        self.left_pwm_pub.publish(self.left_pwm)
        self.right_pwm_pub.publish(self.right_pwm)

    def get_pwm(self, left_speed, right_speed):
        lspeedPWM = max(min((left_speed / self.max_speed) * self.max_pwm_val, self.max_pwm_val), self.min_pwm_val)
        rspeedPWM = max(min((right_speed / self.max_speed) * self.max_pwm_val, self.max_pwm_val), self.min_pwm_val)

        return lspeedPWM, rspeedPWM
    
    def callback(self, data):
        linear_vel = data.linear.x  # Linear Velocity of Robot
        angular_vel = data.angular.z  # Angular Velocity of Robot

        right_vel = linear_vel + (angular_vel * self.wheel_separation) / 2  # right wheel velocity
        left_vel = linear_vel - (angular_vel * self.wheel_separation) / 2  # left wheel velocity

        left_pwm_data, right_pwm_data = self.get_pwm(left_vel, right_vel)

        self.left_pwm.data = int(left_pwm_data)
        self.right_pwm.data = int(right_pwm_data)

        self.left_pwm_pub.publish(self.left_pwm)
        self.right_pwm_pub.publish(self.right_pwm)

def main(args=None):
    rclpy.init(args=args)
    differential_driver = DifferentialDriver()

    rclpy.spin(differential_driver)

    differential_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
