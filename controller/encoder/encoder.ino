#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>

#define LEFT_ENC_PIN_A 18
#define LEFT_ENC_PIN_B 19
#define RIGHT_ENC_PIN_A 21
#define RIGHT_ENC_PIN_B 22

// Encoder values
volatile long long leftEncoderValue = 0;
volatile long rightEncoderValue = 0;

// ROS publisher variables
rcl_publisher_t left_encoder_publisher;
rcl_publisher_t right_encoder_publisher;
std_msgs__msg__Int32 left_encoder_msg;
std_msgs__msg__Int32 right_encoder_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

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

void setup() {
  Serial.begin(115200);

  // Initialize encoder pins
  pinMode(LEFT_ENC_PIN_A, INPUT);
  pinMode(LEFT_ENC_PIN_B, INPUT);
  pinMode(RIGHT_ENC_PIN_A, INPUT);
  pinMode(RIGHT_ENC_PIN_B, INPUT);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), updateLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A), updateRightEncoder, RISING);

  // Initialize micro-ROS
  // set_microros_wifi_transports("A.T.O.M_Labs", "atom281121", "192.168.100.11", 8888); // Replace with your micro-ROS agent IP and port
  set_microros_wifi_transports("The Pot", "passworf", "192.168.2.105", 8888);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_node", "", &support);

  // Initialize publishers
  rclc_publisher_init_default(&left_encoder_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "left_encoder_ticks");
  rclc_publisher_init_default(&right_encoder_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "right_encoder_ticks");

  // Initialize messages
  left_encoder_msg.data = 0;
  right_encoder_msg.data = 0;
}

void loop() {
  left_encoder_msg.data = leftEncoderValue;
  right_encoder_msg.data = rightEncoderValue;

  Serial.print("left_encoder: ");
  Serial.println(leftEncoderValue);
  Serial.print("right_encoder: ");
  Serial.println(rightEncoderValue);

  // Publish encoder data
  rcl_publish(&left_encoder_publisher, &left_encoder_msg, NULL);
  rcl_publish(&right_encoder_publisher, &right_encoder_msg, NULL);

  delay(1000);
}
