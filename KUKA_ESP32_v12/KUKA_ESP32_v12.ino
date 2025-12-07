#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <ESP32Servo.h>

#define ARRAY_SIZE 5

#define FEED_SERVO_1 1
#define FEED_SERVO_2 2
#define FEED_SERVO_3 3
#define FEED_SERVO_4 4
#define FEED_SERVO_GARRA 5

#define CTRL_SERVO_1 11
#define CTRL_SERVO_2 12
#define CTRL_SERVO_3 13
#define CTRL_SERVO_4 14
#define CTRL_SERVO_GARRA 15

Servo servos[ARRAY_SIZE];
int servo_pins[ARRAY_SIZE] = { CTRL_SERVO_1, CTRL_SERVO_2, CTRL_SERVO_3, CTRL_SERVO_4, CTRL_SERVO_GARRA };
int feed_pins[ARRAY_SIZE] = { FEED_SERVO_1, FEED_SERVO_2, FEED_SERVO_3, FEED_SERVO_4, FEED_SERVO_GARRA };

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

std_msgs__msg__Float32MultiArray recv_msg;
std_msgs__msg__Float32MultiArray send_msg;

float recv_data[ARRAY_SIZE];
float current_rad[ARRAY_SIZE];
float target_rad[ARRAY_SIZE];
float send_data[ARRAY_SIZE];

const float STEP_RAD = 0.02 * 5;
const int LOOP_MS = 20;

float rad2deg(float rad) {
  return rad * 180.0 / M_PI;
}
float deg2rad(float deg) {
  return deg * M_PI / 180.0;
}
float saturate(float val, float minv, float maxv) {
  if (val < minv) return minv;
  if (val > maxv) return maxv;
  return val;
}

void subscription_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  for (int i = 0; i < ARRAY_SIZE; i++) {
    target_rad[i] = msg->data.data[i];
  }
}

void setup() {
  set_microros_transports();
  delay(2000);

  for (int i = 0; i < ARRAY_SIZE; i++) {
    servos[i].attach(servo_pins[i]);
    servos[i].write(0);
    current_rad[i] = 0.0;
    target_rad[i] = 0.0;
  }

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_joint_mirror_node", "", &support);

  rclc_publisher_init_default(
    &publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/joint_angles_esp32");

  rclc_subscription_init_default(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/joint_cmd_esp32");

  recv_msg.data.data = recv_data;
  recv_msg.data.size = ARRAY_SIZE;
  recv_msg.data.capacity = ARRAY_SIZE;

  send_msg.data.data = send_data;
  send_msg.data.size = ARRAY_SIZE;
  send_msg.data.capacity = ARRAY_SIZE;

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA);
}

void loop() {
  rclc_executor_spin_some(&executor, 0);

  static unsigned long last_update = 0;
  unsigned long now = millis();

  if (now - last_update >= LOOP_MS) {
    last_update = now;

    for (int i = 0; i < ARRAY_SIZE; i++) {

      float diff = target_rad[i] - current_rad[i];

      if (i == ARRAY_SIZE - 1) {
        current_rad[i] = target_rad[i];
      } else {
        if (diff > STEP_RAD) current_rad[i] += STEP_RAD;
        else if (diff < -STEP_RAD) current_rad[i] -= STEP_RAD;
        else current_rad[i] = target_rad[i];
      }

      float deg = rad2deg(current_rad[i]);
      deg = saturate(deg, 0.0, 180.0);
      servos[i].write((int)deg);

      int adc = analogRead(feed_pins[i]);
      float v = adc * (3.3 / 4095.0);
      float ang = saturate((v / 3.3) * 180.0, 0.0, 180.0);
      send_data[i] = deg2rad(ang);
    }

    rcl_publish(&publisher, &send_msg, NULL);
  }
}
