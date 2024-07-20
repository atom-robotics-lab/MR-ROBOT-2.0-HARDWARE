#include <micro_ros_arduino.h>
#include <ezButton.h>  // the library to use for SW pin
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>

#define CLK_PIN 25 // ESP32 pin GPIO25 connected to the rotary encoder's CLK pin
#define DT_PIN  26 // ESP32 pin GPIO26 connected to the rotary encoder's DT pin
#define SW_PIN  27 // ESP32 pin GPIO27 connected to the rotary encoder's SW pin

#define DIRECTION_CW  0   // clockwise direction
#define DIRECTION_CCW 1  // counter-clockwise direction

int counter = 0;
int direction = DIRECTION_CW;
int CLK_state;
int prev_CLK_state;

ezButton button(SW_PIN);  // create ezButton object that attach to pin 7;

rcl_publisher_t counter_publisher;  // Publisher for encoder counter data
rcl_publisher_t direction_publisher;  // Publisher for encoder direction data

std_msgs__msg__Int32 counter_msg;  // Message for counter data
std_msgs__msg__String direction_msg;  // Message for direction data

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
  pinMode(CLK_PIN, INPUT);
  pinMode(DT_PIN, INPUT);
  button.setDebounceTime(50);  // set debounce time to 50 milliseconds

  // read the initial state of the rotary encoder's CLK pin
  prev_CLK_state = digitalRead(CLK_PIN);

  delay(1000);

  allocator = rcl_get_default_allocator();
  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "rotary_encoder_node", "", &support));

  // create publishers
  RCCHECK(rclc_publisher_init_default(
    &counter_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/rotary_encoder/counter"));

  RCCHECK(rclc_publisher_init_default(
    &direction_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/rotary_encoder/direction"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
}
void loop() {
  button.loop();  // MUST call the loop() function first

  // read the current state of the rotary encoder's CLK pin
  CLK_state = digitalRead(CLK_PIN);

  // If the state of CLK is changed, then pulse occurred
  // React to only the rising edge (from LOW to HIGH) to avoid double count
  if (CLK_state != prev_CLK_state && CLK_state == HIGH) {
    // if the DT state is HIGH
    // the encoder is rotating in counter-clockwise direction => decrease the counter
    if (digitalRead(DT_PIN) == HIGH) {
      counter--;
      direction = DIRECTION_CCW;
    } else {
      // the encoder is rotating in clockwise direction => increase the counter
      counter++;
      direction = DIRECTION_CW;
    }
    // Publish counter value
    counter_msg.data = counter;
    RCCHECK(rcl_publish(&counter_publisher, &counter_msg, NULL));

    // Publish direction value
    const char* direction_str = (direction == DIRECTION_CW) ? "Clockwise" : "Counter-clockwise";
    direction_msg.data.data = (char*) malloc(strlen(direction_str) + 1);
    strcpy(direction_msg.data.data, direction_str);
    direction_msg.data.size = strlen(direction_str);
    direction_msg.data.capacity = direction_msg.data.size + 1;
    RCCHECK(rcl_publish(&direction_publisher, &direction_msg, NULL));
    free(direction_msg.data.data);  // Free the allocated memory
  }
  // save last CLK state
  prev_CLK_state = CLK_state;

  // if (button.isPressed()) {
  //   Serial.println("The button is pressed");
  // }

  // Spin executor
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}