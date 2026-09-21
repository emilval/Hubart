#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <sensor_msgs/msg/joint_state.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// PCA9685 Driver Objects
Adafruit_PWMServoDriver pwm50(0x40);   // 50 Hz Board
Adafruit_PWMServoDriver pwm330(0x41);  // 330 Hz Board

const int PERIOD_50HZ = 20000;
const int PERIOD_330HZ = 3030;
const int STEP_DELAY_MS = 15;
const int HOME_OFFSET = 90;
const int NUM_JOINTS = 6;

struct Joint {
  const char* name;
  Adafruit_PWMServoDriver* driver;
  int channel;
  int periodUs;
  int minPulseUs;
  int maxPulseUs;
  int currentAngle; // Physical 0 - 180 degrees
  int targetAngle;  // Physical 0 - 180 degrees
};

Joint joints[NUM_JOINTS] = {
  {"base_joint",       &pwm330, 0, PERIOD_330HZ, 630,  2690, 90, 90},
  {"shoulder_joint",   &pwm330, 1, PERIOD_330HZ, 610,  670,  90, 90},
  {"elbow_joint",      &pwm50,  2, PERIOD_50HZ,  450,  2480, 90, 90},
  {"neck_joint",       &pwm50,  0, PERIOD_50HZ,  770,  2270, 90, 90},
  {"head_turn_joint",  &pwm50,  1, PERIOD_50HZ,  460,  2440, 90, 90},
  {"gripper_joint",    &pwm50,  4, PERIOD_50HZ,  780,  1820, 90, 90}
};

// Micro-ROS Structures
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_publisher_t joint_state_publisher;
rcl_subscription_t joint_command_subscriber;

sensor_msgs__msg__JointState joint_state_msg;
std_msgs__msg__Float32MultiArray joint_command_msg;

// Pre-allocated memory buffers for Micro-ROS
double position_buffer[NUM_JOINTS];
float command_buffer[NUM_JOINTS];

void setJointAngle(Joint &j, int angleDeg) {
  angleDeg = constrain(angleDeg, 0, 180);
  int centerPulseUs = (j.minPulseUs + j.maxPulseUs) / 2;
  int pulseUs = (angleDeg <= 90) ? map(angleDeg, 0, 90, j.minPulseUs, centerPulseUs)
                                 : map(angleDeg, 90, 180, centerPulseUs, j.maxPulseUs);
  int pwmValue = map(pulseUs, 0, j.periodUs, 0, 4096);
  j.driver->setPWM(j.channel, 0, pwmValue);
}

// Callback when ROS 2 receives new target angles
void joint_command_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  if (msg->data.size >= NUM_JOINTS) {
    for (int i = 0; i < NUM_JOINTS; i++) {
      int userAngle = constrain((int)msg->data.data[i], -90, 90);
      joints[i].targetAngle = userAngle + HOME_OFFSET;
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize hardware PWM drivers
  pwm50.begin();
  pwm50.setPWMFreq(50);
  pwm330.begin();
  pwm330.setPWMFreq(330);
  delay(100);

  for (int i = 0; i < NUM_JOINTS; i++) {
    setJointAngle(joints[i], joints[i].currentAngle);
  }

  // Initialize Micro-ROS Serial Transport
  set_microros_transports();
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create ROS 2 Node
  rclc_node_init_default(&node, "hubert_esp32_node", "", &support);

  // Allocate memory for messages
  joint_command_msg.data.capacity = NUM_JOINTS;
  joint_command_msg.data.size = 0;
  joint_command_msg.data.data = command_buffer;

  joint_state_msg.position.capacity = NUM_JOINTS;
  joint_state_msg.position.size = NUM_JOINTS;
  joint_state_msg.position.data = position_buffer;

  // Initialize Subscriber and Publisher
  rclc_subscription_init_default(
    &joint_command_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/joint_commands"
  );

  rclc_publisher_init_default(
    &joint_state_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/joint_states"
  );

  // Create Executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &joint_command_subscriber, &joint_command_msg, &joint_command_callback, ON_NEW_DATA);
}

void moveJointsSimultaneously() {
  for (int i = 0; i < NUM_JOINTS; i++) {
    if (joints[i].currentAngle < joints[i].targetAngle) {
      joints[i].currentAngle++;
      setJointAngle(joints[i], joints[i].currentAngle);
    } else if (joints[i].currentAngle > joints[i].targetAngle) {
      joints[i].currentAngle--;
      setJointAngle(joints[i], joints[i].currentAngle);
    }
  }
}

void publishJointStates() {
  // Convert current physical angles (0-180) back to relative radians
  for (int i = 0; i < NUM_JOINTS; i++) {
    int userAngleDeg = joints[i].currentAngle - HOME_OFFSET;
    joint_state_msg.position.data[i] = userAngleDeg * (3.14159265 / 180.0);
  }
  rcl_publish(&joint_state_publisher, &joint_state_msg, NULL);
}

unsigned long last_pub_time = 0;

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  moveJointsSimultaneously();

  // Publish /joint_states at 10Hz
  if (millis() - last_pub_time > 100) {
    publishJointStates();
    last_pub_time = millis();
  }
  
  delay(STEP_DELAY_MS);
}