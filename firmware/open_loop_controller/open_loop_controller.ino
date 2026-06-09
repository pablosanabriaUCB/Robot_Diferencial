/*
 * open_loop_controller.ino
 * 
 * Open-loop differential drive robot using micro-ROS over WiFi.
 * Hardware: ESP32 (30-pin) + TB6612FNG motor driver
 * 
 * This is a simpler version that maps cmd_vel directly to PWM
 * without PID control or encoder feedback.
 * 
 * Features:
 *   - State machine reconnection (no rmw_uros_ping_agent)
 *   - Direct cmd_vel to PWM mapping
 *   - Executor only has subscription (1 handle)
 *   - CMD_TIMEOUT safety stop
 *   - Debug prints
 */

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// ============================================================
// WiFi Configuration
// ============================================================
const char* ssid       = "OnLineHS";
const char* password   = "0202-totines";
const char* agent_ip   = "10.0.1.154";
const uint  agent_port = 8888;

// ============================================================
// Motor Driver Pins (TB6612FNG)
// ============================================================
#define STBY  26

#define PWMA  25
#define AIN1  32
#define AIN2  33

#define PWMB  13
#define BIN1  14
#define BIN2  12

// ============================================================
// Motor Constants
// ============================================================
#define MAX_PWM      150   // limit max PWM for open-loop safety
#define CMD_TIMEOUT  500   // ms before stopping on no cmd_vel

// ============================================================
// PWM Channel Configuration (ESP32 LEDC)
// ============================================================
#define PWM_FREQ   5000
#define PWM_RES    8
#define PWM_CH_A   0
#define PWM_CH_B   1

// ============================================================
// micro-ROS State Machine
// ============================================================
enum AgentState {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};

AgentState state = WAITING_AGENT;

// ============================================================
// micro-ROS Objects
// ============================================================
rcl_allocator_t       allocator;
rclc_support_t        support;
rcl_node_t            node;
rcl_subscription_t    subscriber;
rclc_executor_t       executor;
geometry_msgs__msg__Twist msg;

// ============================================================
// Motor Command State
// ============================================================
float cmd_linear  = 0.0;
float cmd_angular = 0.0;

unsigned long last_cmd_time   = 0;
unsigned long last_debug_time = 0;

// ============================================================
// cmd_vel Subscription Callback
// ============================================================
void cmd_vel_callback(const void* msgin) {
  const geometry_msgs__msg__Twist* twist = (const geometry_msgs__msg__Twist*)msgin;

  cmd_linear  = twist->linear.x;
  cmd_angular = twist->angular.z;

  last_cmd_time = millis();
}

// ============================================================
// Motor Control Functions
// ============================================================
void set_motor_L(int pwm_val) {
  if (pwm_val > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else if (pwm_val < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    pwm_val = -pwm_val;
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
  if (pwm_val > MAX_PWM) pwm_val = MAX_PWM;
  ledcWrite(PWM_CH_A, pwm_val);
}

void set_motor_R(int pwm_val) {
  if (pwm_val > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else if (pwm_val < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    pwm_val = -pwm_val;
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
  if (pwm_val > MAX_PWM) pwm_val = MAX_PWM;
  ledcWrite(PWM_CH_B, pwm_val);
}

void stop_motors() {
  set_motor_L(0);
  set_motor_R(0);
  cmd_linear  = 0.0;
  cmd_angular = 0.0;
}

// ============================================================
// Open-Loop Motor Update
// Maps cmd_vel (linear.x, angular.z) directly to PWM values
// ============================================================
void update_motors() {
  // Map velocity commands directly to PWM
  int base_pwm = (int)(cmd_linear  * MAX_PWM);
  int turn_pwm = (int)(cmd_angular * MAX_PWM);

  // Differential drive mixing
  int pwm_L = base_pwm - turn_pwm;
  int pwm_R = base_pwm + turn_pwm;

  // Clamp to valid range
  pwm_L = constrain(pwm_L, -MAX_PWM, MAX_PWM);
  pwm_R = constrain(pwm_R, -MAX_PWM, MAX_PWM);

  set_motor_L(pwm_L);
  set_motor_R(pwm_R);
}

// ============================================================
// micro-ROS Lifecycle: Create & Destroy
// ============================================================
bool create_entities() {
  allocator = rcl_get_default_allocator();

  // Create init options and support
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) return false;

  if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) {
    rcl_init_options_fini(&init_options);
    return false;
  }
  rcl_init_options_fini(&init_options);

  // Create node
  if (rclc_node_init_default(&node, "esp32_openloop_robot", "", &support) != RCL_RET_OK) return false;

  // Create subscription to cmd_vel
  if (rclc_subscription_init_default(
        &subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel") != RCL_RET_OK) return false;

  // Create executor with 1 handle (subscription only)
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;

  if (rclc_executor_add_subscription(
        &executor, &subscriber, &msg,
        &cmd_vel_callback, ON_NEW_DATA) != RCL_RET_OK) return false;

  Serial.println("[uROS] Entities created successfully");
  return true;
}

void destroy_entities() {
  rcl_subscription_fini(&subscriber, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  Serial.println("[uROS] Entities destroyed");
}

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n==============================================");
  Serial.println(" ESP32 Open-Loop Robot - micro-ROS Controller");
  Serial.println("==============================================");

  // Motor pins
  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  digitalWrite(STBY, HIGH);  // Enable motor driver

  // PWM setup
  ledcSetup(PWM_CH_A, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMA, PWM_CH_A);
  ledcAttachPin(PWMB, PWM_CH_B);

  // micro-ROS WiFi transport
  set_microros_wifi_transports((char*)ssid, (char*)password,
                                (char*)agent_ip, agent_port);

  Serial.printf("[WiFi] Connecting to %s ...\n", ssid);
  Serial.printf("[uROS] Agent: %s:%d\n", agent_ip, agent_port);

  stop_motors();
  last_cmd_time   = millis();
  last_debug_time = millis();

  state = WAITING_AGENT;
}

// ============================================================
// Main Loop
// ============================================================
void loop() {
  unsigned long now = millis();

  // --- State Machine for micro-ROS connection ---
  switch (state) {
    case WAITING_AGENT:
      // Try to create entities; if it works, agent is available
      if (create_entities()) {
        state = AGENT_CONNECTED;
        Serial.println("[STATE] -> AGENT_CONNECTED");
      } else {
        // Wait before retrying
        delay(500);
      }
      break;

    case AGENT_CONNECTED:
      // Spin executor (non-blocking, short timeout)
      {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        // Check cmd_vel timeout
        if ((now - last_cmd_time) > CMD_TIMEOUT) {
          cmd_linear  = 0.0;
          cmd_angular = 0.0;
        }

        // Update motors directly from cmd_vel
        update_motors();

        // Debug prints every 500ms
        if ((now - last_debug_time) >= 500) {
          last_debug_time = now;
          Serial.printf("[DBG] linear=%.3f angular=%.3f\n",
                        cmd_linear, cmd_angular);
        }
      }
      break;

    case AGENT_DISCONNECTED:
      // Clean up and go back to waiting
      stop_motors();
      destroy_entities();
      state = WAITING_AGENT;
      Serial.println("[STATE] -> WAITING_AGENT (reconnecting...)");
      delay(500);
      break;

    default:
      break;
  }
}
