#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <rclc/executor.h>

#define LEFT_ENC_PIN_A 18
#define LEFT_ENC_PIN_B 19
#define RIGHT_ENC_PIN_A 21
#define RIGHT_ENC_PIN_B 22

#define LEFT_MOTOR_PWM_PIN 5
#define LEFT_MOTOR_DIR_PIN1 16
#define LEFT_MOTOR_DIR_PIN2 17
#define RIGHT_MOTOR_PWM_PIN 14
#define RIGHT_MOTOR_DIR_PIN1 27
#define RIGHT_MOTOR_DIR_PIN2 26

// Encoder values
volatile long long leftEncoderValue = 0;
volatile long rightEncoderValue = 0;

// PWM values
volatile int32_t leftPwmValue = 0;
volatile int32_t rightPwmValue = 0;

// ROS publisher variables
rcl_publisher_t left_encoder_publisher;
rcl_publisher_t right_encoder_publisher;
std_msgs__msg__Int32 left_encoder_msg;
std_msgs__msg__Int32 right_encoder_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ROS subscriber variables
rcl_subscription_t left_pwm_subscriber;
rcl_subscription_t right_pwm_subscriber;
std_msgs__msg__Int32 left_pwm_msg;
std_msgs__msg__Int32 right_pwm_msg;
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
void leftPwmCallback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  leftPwmValue = msg->data;
  analogWrite(LEFT_MOTOR_PWM_PIN, abs(leftPwmValue));
  if (leftPwmValue > 0) {
    digitalWrite(LEFT_MOTOR_DIR_PIN1, HIGH);
    digitalWrite(LEFT_MOTOR_DIR_PIN2, LOW);
  } else {
    digitalWrite(LEFT_MOTOR_DIR_PIN1, LOW);
    digitalWrite(LEFT_MOTOR_DIR_PIN2, HIGH);
  }
}

void rightPwmCallback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  rightPwmValue = msg->data;
  analogWrite(RIGHT_MOTOR_PWM_PIN, abs(rightPwmValue));
  if (rightPwmValue > 0) {
    digitalWrite(RIGHT_MOTOR_DIR_PIN1, HIGH);
    digitalWrite(RIGHT_MOTOR_DIR_PIN2, LOW);
  } else {
    digitalWrite(RIGHT_MOTOR_DIR_PIN1, LOW);
    digitalWrite(RIGHT_MOTOR_DIR_PIN2, HIGH);
  }
}

void setup() {
  Serial.begin(115200);
  set_microros_wifi_transports("The Pot", "passworf", "192.168.2.105", 8888);

  // Initialize encoder pins
  pinMode(LEFT_ENC_PIN_A, INPUT);
  pinMode(LEFT_ENC_PIN_B, INPUT);
  pinMode(RIGHT_ENC_PIN_A, INPUT);
  pinMode(RIGHT_ENC_PIN_B, INPUT);

  // Initialize motor control pins
  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_DIR_PIN1, OUTPUT);
  pinMode(LEFT_MOTOR_DIR_PIN2, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR_PIN1, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR_PIN2, OUTPUT);

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

  // Initialize subscribers
  rclc_subscription_init_default(&left_pwm_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "left_pwm");
  rclc_subscription_init_default(&right_pwm_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "right_pwm");
  // Initialize executor
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &left_pwm_subscriber, &left_pwm_msg, &leftPwmCallback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &right_pwm_subscriber, &right_pwm_msg, &rightPwmCallback, ON_NEW_DATA);
  // Initialize messages
  left_encoder_msg.data = 0;
  right_encoder_msg.data = 0;
}

void loop() {
  // Spin the node to handle callbacks
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

  // Update and publish encoder data
  left_encoder_msg.data = leftEncoderValue;
  right_encoder_msg.data = rightEncoderValue;

  Serial.print("left_encoder: ");
  Serial.println(leftEncoderValue);
  Serial.print("right_encoder: ");
  Serial.println(rightEncoderValue);

  rcl_publish(&left_encoder_publisher, &left_encoder_msg, NULL);
  rcl_publish(&right_encoder_publisher, &right_encoder_msg, NULL);

  delay(100);
}
