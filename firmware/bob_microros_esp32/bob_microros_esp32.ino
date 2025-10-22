/*
 * BOB Autonomous Mower - ESP32 Micro-ROS Firmware
 *
 * Hardware Configuration:
 * - ESP32-WROOM-32 (Arduino compatible)
 * - L298N Motor Driver (GPIO 25,26,27,33)
 * - BNO086 IMU via I2C (address 0x4B, SDA=21, SCL=22)
 * - Wheel Encoders (GPIO 32 left, GPIO 35 right)
 * - Differential Drive: 250x160mm wheelbase, 67mm diameter wheels
 * - Encoder: 20 pulses/revolution, 413 pulses/meter
 *
 * ROS2 Topics:
 * - Subscribes: /cmd_vel (geometry_msgs/Twist)
 * - Publishes: /odom (nav_msgs/Odometry)
 * - Publishes: /imu (sensor_msgs/Imu)
 */

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <Adafruit_BNO08x.h>

// ==================== HARDWARE PIN DEFINITIONS ====================
// Motor Driver L298N
#define MOTOR_LEFT_PWM    25    // Left motor PWM
#define MOTOR_LEFT_DIR1   26    // Left motor direction pin 1
#define MOTOR_LEFT_DIR2   27    // Left motor direction pin 2
#define MOTOR_RIGHT_PWM   33    // Right motor PWM
#define MOTOR_RIGHT_DIR1  32    // Right motor direction pin 1 (shared with encoder)
#define MOTOR_RIGHT_DIR2  14    // Right motor direction pin 2

// Encoders (NOTE: GPIO 32 and 35 are INPUT_ONLY on ESP32)
#define ENCODER_LEFT      34    // Left encoder (GPIO 34 is input-only, better for encoder)
#define ENCODER_RIGHT     35    // Right encoder (GPIO 35 is input-only)

// I2C for BNO086 IMU
#define I2C_SDA           21    // Default SDA
#define I2C_SCL           22    // Default SCL

// ==================== ROBOT PHYSICAL PARAMETERS ====================
#define WHEEL_BASE        0.250  // meters (250mm)
#define WHEEL_DIAMETER    0.067  // meters (67mm)
#define WHEEL_RADIUS      (WHEEL_DIAMETER / 2.0)
#define PULSES_PER_REV    20     // Encoder pulses per wheel revolution
#define PULSES_PER_METER  413    // As specified

// ==================== MICRO-ROS CONFIGURATION ====================
rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_subscription_t cmd_vel_subscriber;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

// ==================== ODOMETRY VARIABLES ====================
volatile long encoder_left_count = 0;
volatile long encoder_right_count = 0;
long last_encoder_left = 0;
long last_encoder_right = 0;

float robot_x = 0.0;
float robot_y = 0.0;
float robot_theta = 0.0;

unsigned long last_odom_time = 0;

// ==================== IMU CONFIGURATION ====================
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

// ==================== MOTOR CONTROL ====================
float target_linear_vel = 0.0;  // m/s
float target_angular_vel = 0.0; // rad/s

// PWM Configuration
const int pwmFreq = 5000;
const int pwmResolution = 8;
const int pwmChannelLeft = 0;
const int pwmChannelRight = 1;

// ==================== ERROR HANDLING MACRO ====================
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while(1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// ==================== ENCODER ISR ====================
void IRAM_ATTR encoder_left_isr() {
  encoder_left_count++;
}

void IRAM_ATTR encoder_right_isr() {
  encoder_right_count++;
}

// ==================== CMD_VEL CALLBACK ====================
void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  target_linear_vel = msg->linear.x;
  target_angular_vel = msg->angular.z;

  // Apply motor commands
  apply_motor_control(target_linear_vel, target_angular_vel);
}

// ==================== MOTOR CONTROL FUNCTIONS ====================
void setup_motors() {
  // Configure PWM channels
  ledcSetup(pwmChannelLeft, pwmFreq, pwmResolution);
  ledcSetup(pwmChannelRight, pwmFreq, pwmResolution);

  // Attach channels to GPIO pins
  ledcAttachPin(MOTOR_LEFT_PWM, pwmChannelLeft);
  ledcAttachPin(MOTOR_RIGHT_PWM, pwmChannelRight);

  // Direction pins
  pinMode(MOTOR_LEFT_DIR1, OUTPUT);
  pinMode(MOTOR_LEFT_DIR2, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR1, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR2, OUTPUT);

  // Start with motors stopped
  stop_motors();
}

void stop_motors() {
  ledcWrite(pwmChannelLeft, 0);
  ledcWrite(pwmChannelRight, 0);
  digitalWrite(MOTOR_LEFT_DIR1, LOW);
  digitalWrite(MOTOR_LEFT_DIR2, LOW);
  digitalWrite(MOTOR_RIGHT_DIR1, LOW);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
}

void apply_motor_control(float linear_vel, float angular_vel) {
  // Differential drive kinematics
  // v_left = linear_vel - (angular_vel * WHEEL_BASE / 2)
  // v_right = linear_vel + (angular_vel * WHEEL_BASE / 2)

  float v_left = linear_vel - (angular_vel * WHEEL_BASE / 2.0);
  float v_right = linear_vel + (angular_vel * WHEEL_BASE / 2.0);

  // Convert velocities to PWM (simple mapping, needs calibration)
  // Max velocity ~0.5 m/s = PWM 255
  int pwm_left = constrain(abs(v_left) * 510, 0, 255);  // 510 = 255 / 0.5
  int pwm_right = constrain(abs(v_right) * 510, 0, 255);

  // Left motor
  if (v_left >= 0) {
    digitalWrite(MOTOR_LEFT_DIR1, HIGH);
    digitalWrite(MOTOR_LEFT_DIR2, LOW);
  } else {
    digitalWrite(MOTOR_LEFT_DIR1, LOW);
    digitalWrite(MOTOR_LEFT_DIR2, HIGH);
  }
  ledcWrite(pwmChannelLeft, pwm_left);

  // Right motor
  if (v_right >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
    digitalWrite(MOTOR_RIGHT_DIR2, LOW);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR1, LOW);
    digitalWrite(MOTOR_RIGHT_DIR2, HIGH);
  }
  ledcWrite(pwmChannelRight, pwm_right);
}

// ==================== ENCODER SETUP ====================
void setup_encoders() {
  pinMode(ENCODER_LEFT, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), encoder_left_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), encoder_right_isr, RISING);
}

// ==================== IMU SETUP ====================
bool setup_imu() {
  Wire.begin(I2C_SDA, I2C_SCL);

  if (!bno08x.begin_I2C(0x4B)) {
    return false;
  }

  // Enable rotation vector and accelerometer reports
  if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 10000)) {  // 100Hz = 10000 us
    return false;
  }
  if (!bno08x.enableReport(SH2_ACCELEROMETER, 10000)) {
    return false;
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000)) {
    return false;
  }

  return true;
}

// ==================== ODOMETRY COMPUTATION ====================
void compute_odometry() {
  unsigned long current_time = millis();
  float dt = (current_time - last_odom_time) / 1000.0;  // Convert to seconds

  if (dt <= 0) return;

  // Get encoder deltas
  long delta_left = encoder_left_count - last_encoder_left;
  long delta_right = encoder_right_count - last_encoder_right;

  last_encoder_left = encoder_left_count;
  last_encoder_right = encoder_right_count;

  // Convert encoder counts to distance
  float dist_left = (delta_left / (float)PULSES_PER_METER);
  float dist_right = (delta_right / (float)PULSES_PER_METER);

  // Compute center distance and rotation
  float dist_center = (dist_left + dist_right) / 2.0;
  float delta_theta = (dist_right - dist_left) / WHEEL_BASE;

  // Update pose
  robot_x += dist_center * cos(robot_theta + delta_theta / 2.0);
  robot_y += dist_center * sin(robot_theta + delta_theta / 2.0);
  robot_theta += delta_theta;

  // Normalize theta to [-pi, pi]
  while (robot_theta > PI) robot_theta -= 2 * PI;
  while (robot_theta < -PI) robot_theta += 2 * PI;

  // Compute velocities
  float v_linear = dist_center / dt;
  float v_angular = delta_theta / dt;

  // Populate odometry message
  odom_msg.header.stamp.sec = current_time / 1000;
  odom_msg.header.stamp.nanosec = (current_time % 1000) * 1000000;
  odom_msg.header.frame_id.data = "odom";
  odom_msg.child_frame_id.data = "base_link";

  // Position
  odom_msg.pose.pose.position.x = robot_x;
  odom_msg.pose.pose.position.y = robot_y;
  odom_msg.pose.pose.position.z = 0.0;

  // Orientation (quaternion from theta)
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sin(robot_theta / 2.0);
  odom_msg.pose.pose.orientation.w = cos(robot_theta / 2.0);

  // Velocity
  odom_msg.twist.twist.linear.x = v_linear;
  odom_msg.twist.twist.angular.z = v_angular;

  last_odom_time = current_time;

  // Publish odometry
  RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}

// ==================== IMU READING ====================
void read_and_publish_imu() {
  if (bno08x.wasReset()) {
    setup_imu();
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  unsigned long current_time = millis();

  switch (sensorValue.sensorId) {
    case SH2_ARVR_STABILIZED_RV:
      // Quaternion orientation
      imu_msg.orientation.x = sensorValue.un.arvrStabilizedRV.i;
      imu_msg.orientation.y = sensorValue.un.arvrStabilizedRV.j;
      imu_msg.orientation.z = sensorValue.un.arvrStabilizedRV.k;
      imu_msg.orientation.w = sensorValue.un.arvrStabilizedRV.real;
      break;

    case SH2_GYROSCOPE_CALIBRATED:
      // Angular velocity
      imu_msg.angular_velocity.x = sensorValue.un.gyroscope.x;
      imu_msg.angular_velocity.y = sensorValue.un.gyroscope.y;
      imu_msg.angular_velocity.z = sensorValue.un.gyroscope.z;
      break;

    case SH2_ACCELEROMETER:
      // Linear acceleration
      imu_msg.linear_acceleration.x = sensorValue.un.accelerometer.x;
      imu_msg.linear_acceleration.y = sensorValue.un.accelerometer.y;
      imu_msg.linear_acceleration.z = sensorValue.un.accelerometer.z;
      break;
  }

  // Populate header
  imu_msg.header.stamp.sec = current_time / 1000;
  imu_msg.header.stamp.nanosec = (current_time % 1000) * 1000000;
  imu_msg.header.frame_id.data = "imu_link";

  // Publish IMU data
  RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
}

// ==================== TIMER CALLBACK ====================
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    compute_odometry();
    read_and_publish_imu();
  }
}

// ==================== SETUP ====================
void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);

  // LED for status
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Setup hardware
  setup_motors();
  setup_encoders();

  if (!setup_imu()) {
    Serial.println("IMU initialization failed!");
    // Continue anyway - IMU is optional for basic testing
  }

  // Initialize micro-ROS transport
  set_microros_transports();

  delay(2000);

  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "bob_esp32_node", "", &support));

  // Create publishers
  RCCHECK(rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom"));

  RCCHECK(rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu"));

  // Create subscriber
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // Create timer (100Hz = 10ms)
  const unsigned int timer_timeout = 10;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Initialize odometry timestamp
  last_odom_time = millis();

  Serial.println("BOB ESP32 Micro-ROS Node Ready!");
  digitalWrite(LED_BUILTIN, LOW);
}

// ==================== MAIN LOOP ====================
void loop() {
  // Spin executor to handle callbacks
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(10);
}
