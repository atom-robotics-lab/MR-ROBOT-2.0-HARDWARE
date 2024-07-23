#include <micro_ros_arduino.h>
#include <ezButton.h>  // the library to use for SW pin
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>

//pins for motor1
#define CLK_PIN1 25 // ESP32 pin GPIO25 connected to the rotary encoder's CLK pin
#define DT_PIN1  26 // ESP32 pin GPIO26 connected to the rotary encoder's DT pin
#define SW_PIN1 27 // ESP32 pin GPIO27 connected to the rotary encoder's SW pin
//pins for motor2
#define CLK_PIN2 32 // ESP32 pin GPIO25 connected to the rotary encoder's CLK pin
#define DT_PIN2 33 // ESP32 pin GPIO26 connected to the rotary encoder's DT pin
#define SW_PIN2 34 // ESP32 pin GPIO27 connected to the rotary encoder's SW pin

#define DIRECTION_CW  0   // clockwise direction
#define DIRECTION_CCW 1  // counter-clockwise direction

// variables for motor1
int counter1 = 0;
int direction1 = DIRECTION_CW;
int CLK_state1;
int prev_CLK_state1;
ezButton button1(SW_PIN1);  // create ezButton object that attach to pin 7;

rcl_publisher_t counter_publisher1;  // Publisher for encoder counter data
rcl_publisher_t direction_publisher1;  // Publisher for encoder direction data

std_msgs__msg__Int32 counter_msg1;  // Message for counter data
std_msgs__msg__String direction_msg1;  // Message for direction data

// variables for motor2
int counter2 = 0;
int direction2 = DIRECTION_CW;
int CLK_state2;
int prev_CLK_state2;
ezButton button2(SW_PIN2);  // create ezButton object that attach to pin 7;

rcl_publisher_t counter_publisher2;  // Publisher for encoder counter data
rcl_publisher_t direction_publisher2;  // Publisher for encoder direction data

std_msgs__msg__Int32 counter_msg2;  // Message for counter data
std_msgs__msg__String direction_msg2;  // Message for direction data

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop(){
  while(1){
    // Blink LED to indicate error
    delay(100);
  }
}

void setup() {
  set_microros_wifi_transports("ZTE-9PF657", "bk3yakyj", "192.168.1.10", 8888);
  // configure encoder pins as inputs
  pinMode(CLK_PIN1, INPUT);
  pinMode(DT_PIN1, INPUT);
  button1.setDebounceTime(50);  // set debounce time to 50 milliseconds

  pinMode(CLK_PIN2, INPUT);
  pinMode(DT_PIN2, INPUT);
  button2.setDebounceTime(50);  // set debounce time to 50 milliseconds

  // read the initial state of the rotary encoder's CLK pin
  prev_CLK_state1 = digitalRead(CLK_PIN1);
  prev_CLK_state2 = digitalRead(CLK_PIN2);

  delay(1000);

  allocator = rcl_get_default_allocator();
  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "rotary_encoder_node", "", &support));

  // create publishers for motor1
  RCCHECK(rclc_publisher_init_default(
    &counter_publisher1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/rotary_encoder1/counter"));

  RCCHECK(rclc_publisher_init_default(
    &direction_publisher1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/rotary_encoder1/direction"));

  // create publishers for motor2
  RCCHECK(rclc_publisher_init_default(
    &counter_publisher2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/rotary_encoder2/counter"));

  RCCHECK(rclc_publisher_init_default(
    &direction_publisher2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/rotary_encoder2/direction"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
}
void loop() {
  button1.loop();  // MUST call the loop() function first
  button2.loop();

  //for motor1
  // read the current state of the rotary encoder's CLK pin
  CLK_state1 = digitalRead(CLK_PIN1);
  // If the state of CLK is changed, then pulse occurred
  // React to only the rising edge (from LOW to HIGH) to avoid double count
  if (CLK_state1 != prev_CLK_state1 && CLK_state1 == HIGH) {
    // if the DT state is HIGH
    // the encoder is rotating in counter-clockwise direction => decrease the counter
    if (digitalRead(DT_PIN1) == HIGH) {
      counter1--;
      direction1 = DIRECTION_CCW;
    } else {
      // the encoder is rotating in clockwise direction => increase the counter
      counter1++;
      direction1 = DIRECTION_CW;
    }
    // Publish counter value
    counter_msg1.data = counter1;
    RCCHECK(rcl_publish(&counter_publisher1, &counter_msg1, NULL));

    // Publish direction value
    const char* direction_str1 = (direction1 == DIRECTION_CW) ? "Clockwise" : "Counter-clockwise";
    direction_msg1.data.data = (char*) malloc(strlen(direction_str1) + 1);
    strcpy(direction_msg1.data.data, direction_str1);
    direction_msg1.data.size = strlen(direction_str1);
    direction_msg1.data.capacity = direction_msg1.data.size + 1;
    RCCHECK(rcl_publish(&direction_publisher1, &direction_msg1, NULL));
    free(direction_msg1.data.data);  // Free the allocated memory
  }
  // save last CLK state
  prev_CLK_state1 = CLK_state1;

  //for motor2
  // read the current state of the rotary encoder's CLK pin
  CLK_state2 = digitalRead(CLK_PIN2);
  // If the state of CLK is changed, then pulse occurred
  // React to only the rising edge (from LOW to HIGH) to avoid double count
  if (CLK_state2 != prev_CLK_state2 && CLK_state2 == HIGH) {
    // if the DT state is HIGH
    // the encoder is rotating in counter-clockwise direction => decrease the counter
    if (digitalRead(DT_PIN2) == HIGH) {
      counter2--;
      direction2 = DIRECTION_CCW;
    } else {
      // the encoder is rotating in clockwise direction => increase the counter
      counter2++;
      direction2 = DIRECTION_CW;
    }
    // Publish counter value
    counter_msg2.data = counter2;
    RCCHECK(rcl_publish(&counter_publisher2, &counter_msg2, NULL));

    // Publish direction value
    const char* direction_str2 = (direction2 == DIRECTION_CW) ? "Clockwise" : "Counter-clockwise";
    direction_msg2.data.data = (char*) malloc(strlen(direction_str2) + 1);
    strcpy(direction_msg2.data.data, direction_str2);
    direction_msg2.data.size = strlen(direction_str2);
    direction_msg2.data.capacity = direction_msg2.data.size + 1;
    RCCHECK(rcl_publish(&direction_publisher2, &direction_msg2, NULL));
    free(direction_msg2.data.data);  // Free the allocated memory
  }
  // save last CLK state
  prev_CLK_state2 = CLK_state2;

  // Spin executor
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}