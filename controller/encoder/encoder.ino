#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define LEFT_ENC_PIN_A 34
#define LEFT_ENC_PIN_B 23
#define RIGHT_ENC_PIN_A 35
#define RIGHT_ENC_PIN_B 32

#define LEFT_MOTOR_RPWM_PIN 25
#define LEFT_MOTOR_LPWM_PIN 26
#define RIGHT_MOTOR_RPWM_PIN 33
#define RIGHT_MOTOR_LPWM_PIN 5

Adafruit_MPU6050 mpu;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;

rcl_publisher_t left_encoder_pub;
rcl_publisher_t right_encoder_pub;
rcl_publisher_t imu_pub;

rcl_subscription_t pwm_sub;
std_msgs__msg__Int32 pwm_msg;

// Messages
std_msgs__msg__Int32 left_encoder_msg;
std_msgs__msg__Int32 right_encoder_msg;
sensor_msgs__msg__Imu imu_msg;

volatile long long leftEncoderValue = 0;
volatile long long rightEncoderValue = 0;
volatile int32_t leftPwmValue = 0;
volatile int32_t rightPwmValue = 0;

void IRAM_ATTR updateLeftEncoder() {
  if (digitalRead(LEFT_ENC_PIN_A) > digitalRead(LEFT_ENC_PIN_B))
    leftEncoderValue++;
  else
    leftEncoderValue--;
}

void IRAM_ATTR updateRightEncoder() {
  if (digitalRead(RIGHT_ENC_PIN_A) > digitalRead(RIGHT_ENC_PIN_B))
    rightEncoderValue++;
  else
    rightEncoderValue--;
}

void PwmCallback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;

  leftPwmValue = ((msg->data) / 1000) - 255 * 2;
  rightPwmValue = ((msg->data) % 1000) - 255 * 2;

  if (leftPwmValue >= 0) {
    analogWrite(LEFT_MOTOR_RPWM_PIN, abs(leftPwmValue));
    analogWrite(LEFT_MOTOR_LPWM_PIN, 0);
  } else {
    analogWrite(LEFT_MOTOR_RPWM_PIN, 0);
    analogWrite(LEFT_MOTOR_LPWM_PIN, abs(leftPwmValue));
  }

  if (rightPwmValue >= 0) {
    analogWrite(RIGHT_MOTOR_RPWM_PIN, abs(rightPwmValue));
    analogWrite(RIGHT_MOTOR_LPWM_PIN, 0);
  } else {
    analogWrite(RIGHT_MOTOR_RPWM_PIN, 0);
    analogWrite(RIGHT_MOTOR_LPWM_PIN, abs(rightPwmValue));
  }
}

void subscriber_task(void *param) {
  (void)param;
  while (true) {
    rcl_ret_t ret = rcl_take(&pwm_sub, &pwm_msg, NULL, NULL);
    if (ret == RCL_RET_OK)
      PwmCallback(&pwm_msg);
    vTaskDelay(pdMS_TO_TICKS(100)); 
  }
}

void publisher_task(void *param) {
  (void)param;
  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(100);  // 10 Hz

  while (true) {
    // --- Encoder data ---
    left_encoder_msg.data = leftEncoderValue;
    right_encoder_msg.data = -rightEncoderValue; 

    rcl_publish(&left_encoder_pub, &left_encoder_msg, NULL);
    rcl_publish(&right_encoder_pub, &right_encoder_msg, NULL);

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    imu_msg.linear_acceleration.x = a.acceleration.x;
    imu_msg.linear_acceleration.y = a.acceleration.y;
    imu_msg.linear_acceleration.z = a.acceleration.z;

    imu_msg.angular_velocity.x = g.gyro.x;
    imu_msg.angular_velocity.y = g.gyro.y;
    imu_msg.angular_velocity.z = g.gyro.z;

    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 0.0;

    rcl_publish(&imu_pub, &imu_msg, NULL);

    vTaskDelayUntil(&last_wake, period);
  }
}

void setup() {
  Serial.begin(115200);
  set_microros_transports();
  delay(1000);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050!");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  pinMode(LEFT_ENC_PIN_A, INPUT);
  pinMode(LEFT_ENC_PIN_B, INPUT);
  pinMode(RIGHT_ENC_PIN_A, INPUT);
  pinMode(RIGHT_ENC_PIN_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), updateLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A), updateRightEncoder, RISING);

  pinMode(LEFT_MOTOR_RPWM_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_LPWM_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_RPWM_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_LPWM_PIN, OUTPUT);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_freertos_node", "", &support);

  rclc_publisher_init_default(&left_encoder_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "left_encoder_ticks");
  rclc_publisher_init_default(&right_encoder_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "right_encoder_ticks");
  rclc_publisher_init_default(&imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu");

  rclc_subscription_init_default(&pwm_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "pwm");

  xTaskCreatePinnedToCore(subscriber_task, "PWM_Sub_Task", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(publisher_task, "Pub_Task", 4096, NULL, 1, NULL, 1);

  Serial.println("micro-ROS FreeRTOS node started.");
}

void loop() {}
