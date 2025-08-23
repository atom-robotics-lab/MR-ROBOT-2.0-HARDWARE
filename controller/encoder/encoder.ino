#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/int32.h>
// #include <stdlib.h> // for malloc

#include <rclc/executor.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <sensor_msgs/msg/imu.h>

#define LEFT_ENC_PIN_A 34
#define LEFT_ENC_PIN_B 23

#define RIGHT_ENC_PIN_A 35
#define RIGHT_ENC_PIN_B 32

#define LEFT_MOTOR_RPWM_PIN 25
#define LEFT_MOTOR_LPWM_PIN 26

#define RIGHT_MOTOR_RPWM_PIN 33
#define RIGHT_MOTOR_LPWM_PIN 5


Adafruit_MPU6050 mpu; 
sensor_msgs__msg__Imu imu_msg;

// Encoder values
volatile long long leftEncoderValue = 0;
volatile long rightEncoderValue = 0;

// PWM values
volatile int32_t leftPwmValue = 0;
volatile int32_t rightPwmValue = 0;

// ROS publisher variables
rcl_publisher_t left_encoder_publisher;
rcl_publisher_t right_encoder_publisher;

rcl_publisher_t imu_pub;

std_msgs__msg__Int32 left_encoder_msg;
std_msgs__msg__Int32 right_encoder_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ROS subscriber variables
rcl_subscription_t pwm_subscriber;
std_msgs__msg__Int32 pwm_msg;
rclc_executor_t executor;

// Interrupt service routines for encoders
void IRAM_ATTR updateLeftEncoder() {
  if (digitalRead(LEFT_ENC_PIN_A) > digitalRead(LEFT_ENC_PIN_B)) {
    leftEncoderValue++;
  } else {
    leftEncoderValue--;
  }
}

void IRAM_ATTR updateRightEncoder() {
  if (digitalRead(RIGHT_ENC_PIN_A) > digitalRead(RIGHT_ENC_PIN_B)) {
    rightEncoderValue++;
  } else {
    rightEncoderValue--;
  }
}

// Callback functions for PWM subscribers
void PwmCallback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  leftPwmValue = ((msg->data)/1000) - 255*2;
  if(leftPwmValue >= 0 ){
  analogWrite(LEFT_MOTOR_RPWM_PIN, abs(leftPwmValue));
  analogWrite(LEFT_MOTOR_LPWM_PIN, abs(0));}
  else{
    analogWrite(LEFT_MOTOR_RPWM_PIN, abs(0));
  analogWrite(LEFT_MOTOR_LPWM_PIN, abs(leftPwmValue));
  }

  // Serial.println(leftPwmValue);

  rightPwmValue = ((msg->data)%1000) - 255*2;

  if(rightPwmValue >= 0 ){
  analogWrite(RIGHT_MOTOR_RPWM_PIN, abs(rightPwmValue));
  analogWrite(RIGHT_MOTOR_LPWM_PIN, abs(0));}
  else{
    analogWrite(RIGHT_MOTOR_RPWM_PIN, abs(0));
  analogWrite(RIGHT_MOTOR_LPWM_PIN, abs(rightPwmValue));
  }
}

// void rightPwmCallback(const void * msgin) {
//   const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
//   rightPwmValue = msg->data[1];
//   analogWrite(RIGHT_MOTOR_RPWM_PIN, abs(rightPwmValue));
//   analogWrite(RIGHT_MOTOR_LPWM_PIN, abs(0));
// }

void setup() {
  // pwm_msg.data.data = (int32_t*)malloc(2 * sizeof(int32_t)); // for left and right motor
  // pwm_msg.data.size = 2;
  // pwm_msg.data.capacity = 2;

  Serial.begin(115200);
  set_microros_transports();

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  // Initialize encoder pins
  pinMode(LEFT_ENC_PIN_A, INPUT);
  pinMode(LEFT_ENC_PIN_B, INPUT);
  pinMode(RIGHT_ENC_PIN_A, INPUT);
  pinMode(RIGHT_ENC_PIN_B, INPUT);

  // Initialize motor control pins
  pinMode(LEFT_MOTOR_RPWM_PIN, OUTPUT);

  pinMode(RIGHT_MOTOR_RPWM_PIN, OUTPUT);

  pinMode(LEFT_MOTOR_LPWM_PIN, OUTPUT);

  pinMode(RIGHT_MOTOR_LPWM_PIN, OUTPUT);
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
 

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), updateLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A), updateRightEncoder, RISING);

  // Initialize micro-ROS 
  // set_microros_wifi_transports("A.T.O.M_Labs", "atom281121", "192.168.100.11", 8888); 
  // Replace with your micro-ROS agent IP and port 
  // set_microros_transports();
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_node", "", &support);

  // Initialize publishers
  rclc_publisher_init_default(&left_encoder_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "left_encoder_ticks");
  rclc_publisher_init_default(&right_encoder_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "right_encoder_ticks");
  rclc_publisher_init_default(&imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu");

  // Initialize subscribers
  rclc_subscription_init_default(&pwm_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "pwm");

  // Initialize executor
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &pwm_subscriber, &pwm_msg, &PwmCallback, ON_NEW_DATA);
  // Initialize messages
  left_encoder_msg.data = 0;
  right_encoder_msg.data = 0;
}

void loop() {
  // Spin the node to handle callbacks
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

  // Update and publish encoder data
  left_encoder_msg.data = leftEncoderValue;
  right_encoder_msg.data = rightEncoderValue*-1;

  rcl_publish(&right_encoder_publisher, &right_encoder_msg, NULL);
  rcl_publish(&left_encoder_publisher, &left_encoder_msg, NULL);

  // --- IMU Publish ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  imu_msg.linear_acceleration.x = a.acceleration.x;
  imu_msg.linear_acceleration.y = a.acceleration.y;
  imu_msg.linear_acceleration.z = a.acceleration.z;

  imu_msg.angular_velocity.x = g.gyro.x;
  imu_msg.angular_velocity.y = g.gyro.y;
  imu_msg.angular_velocity.z = g.gyro.z;

  // orientation (MPU6050 gyro+accel se direct orientation nahi milta → blank rakho)
  imu_msg.orientation.w = 0.0;
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;

  rcl_publish(&imu_pub, &imu_msg, NULL);

  delay(100);
}
