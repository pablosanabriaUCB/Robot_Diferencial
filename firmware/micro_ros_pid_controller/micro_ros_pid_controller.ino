/*
 * micro_ros_pid_controller.ino
 * 
 * PID-controlled differential drive robot using micro-ROS over WiFi.
 * Hardware: ESP32 (30-pin) + TB6612FNG motor driver + quadrature encoders
 * 
 * Features & Fixes:
 *   - State machine reconnection (no rmw_uros_ping_agent)
 *   - PID runs in loop() via millis(), NOT as executor timer
 *   - Executor only has subscription (1 handle)
 *   - Right encoder inverted (ticks_R-- on HIGH, ticks_R++ on LOW)
 *   - Low-pass filter on encoder velocity (FILTER_ALPHA = 0.15)
 *   - Dead zone compensation on motors (min PWM 60)
 *   - volatile target_speed variables
 *   - Debug prints every 500ms
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
// Encoder Pins
// ============================================================
#define ENC_L_A  34
#define ENC_L_B  35
#define ENC_R_A  23
#define ENC_R_B  19

// ============================================================
// Robot Physical Parameters
// ============================================================
#define WHEEL_BASE    0.20   // meters (distance between wheels)
#define WHEEL_RADIUS  0.02   // meters
#define TICKS_PER_REV 150    // encoder ticks per wheel revolution (calibrado)

// ============================================================
// PID Parameters (final tuned values)
// ============================================================
#define Kp  0.5
#define Ki  0.0
#define Kd  0.01

#define DT             0.05   // 50ms control loop period
#define INTEGRAL_LIMIT 50.0   // anti-windup clamp
#define CMD_TIMEOUT    500    // ms before stopping on no cmd_vel
#define FILTER_ALPHA   0.15   // low-pass filter coefficient for velocity

// ============================================================
// Motor Constants
// ============================================================
#define MAX_PWM     255
#define MIN_PWM      60   // dead zone compensation

// ============================================================
// PWM Channel Configuration (ESP32 LEDC)
// ============================================================
#define PWM_FREQ      5000
#define PWM_RES       8
#define PWM_CH_A      0
#define PWM_CH_B      1

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
// Encoder Variables (volatile for ISR safety)
// ============================================================
volatile long ticks_L = 0;
volatile long ticks_R = 0;

// ============================================================
// PID & Motor State
// ============================================================
volatile float target_speed_L = 0.0;  // target wheel speed (m/s)
volatile float target_speed_R = 0.0;

float current_speed_L = 0.0;  // filtered measured speed (m/s)
float current_speed_R = 0.0;

float integral_L = 0.0, prev_error_L = 0.0;
float integral_R = 0.0, prev_error_R = 0.0;

long prev_ticks_L = 0, prev_ticks_R = 0;

unsigned long last_pid_time   = 0;
unsigned long last_cmd_time   = 0;
unsigned long last_debug_time = 0;

// ============================================================
// Encoder ISRs
// ============================================================
void IRAM_ATTR enc_L_ISR() {
  if (digitalRead(ENC_L_B) == HIGH)
    ticks_L++;
  else
    ticks_L--;
}

// Right encoder is INVERTED
void IRAM_ATTR enc_R_ISR() {
  if (digitalRead(ENC_R_B) == HIGH)
    ticks_R--;   // inverted
  else
    ticks_R++;   // inverted
}

// ============================================================
// cmd_vel Subscription Callback
// ============================================================
void cmd_vel_callback(const void* msgin) {
  const geometry_msgs__msg__Twist* twist = (const geometry_msgs__msg__Twist*)msgin;

  float linear  = twist->linear.x;
  float angular = twist->angular.z;

  // Differential drive kinematics: convert (v, ω) to wheel speeds
  target_speed_L = linear - (angular * WHEEL_BASE / 2.0);
  target_speed_R = linear + (angular * WHEEL_BASE / 2.0);

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
  // Dead zone compensation
  if (pwm_val > 0 && pwm_val < MIN_PWM) pwm_val = MIN_PWM;
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
  // Dead zone compensation
  if (pwm_val > 0 && pwm_val < MIN_PWM) pwm_val = MIN_PWM;
  if (pwm_val > MAX_PWM) pwm_val = MAX_PWM;
  ledcWrite(PWM_CH_B, pwm_val);
}

void stop_motors() {
  set_motor_L(0);
  set_motor_R(0);
  integral_L = 0; prev_error_L = 0;
  integral_R = 0; prev_error_R = 0;
  current_speed_L = 0;
  current_speed_R = 0;
}

// ============================================================
// PID Compute (called from loop at DT intervals)
// ============================================================
void pid_update() {
  // Read encoder ticks atomically
  noInterrupts();
  long tL = ticks_L;
  long tR = ticks_R;
  interrupts();

  // Compute raw velocity from encoder deltas
  long delta_L = tL - prev_ticks_L;
  long delta_R = tR - prev_ticks_R;
  prev_ticks_L = tL;
  prev_ticks_R = tR;

  float dist_per_tick = (2.0 * PI * WHEEL_RADIUS) / TICKS_PER_REV;
  float raw_speed_L = (delta_L * dist_per_tick) / DT;
  float raw_speed_R = (delta_R * dist_per_tick) / DT;

  // Low-pass filter on measured velocity
  current_speed_L = FILTER_ALPHA * raw_speed_L + (1.0 - FILTER_ALPHA) * current_speed_L;
  current_speed_R = FILTER_ALPHA * raw_speed_R + (1.0 - FILTER_ALPHA) * current_speed_R;

  // PID for left wheel
  float error_L = target_speed_L - current_speed_L;
  integral_L += error_L * DT;
  integral_L = constrain(integral_L, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float derivative_L = (error_L - prev_error_L) / DT;
  float output_L = Kp * error_L + Ki * integral_L + Kd * derivative_L;
  prev_error_L = error_L;

  // PID for right wheel
  float error_R = target_speed_R - current_speed_R;
  integral_R += error_R * DT;
  integral_R = constrain(integral_R, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float derivative_R = (error_R - prev_error_R) / DT;
  float output_R = Kp * error_R + Ki * integral_R + Kd * derivative_R;
  prev_error_R = error_R;

  // Convert PID output to PWM
  int pwm_L = (int)(output_L * MAX_PWM);
  int pwm_R = (int)(output_R * MAX_PWM);

  // If target is zero, just stop
  if (target_speed_L == 0.0 && target_speed_R == 0.0) {
    stop_motors();
  } else {
    set_motor_L(pwm_L);
    set_motor_R(pwm_R);
  }
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
  if (rclc_node_init_default(&node, "esp32_pid_robot", "", &support) != RCL_RET_OK) return false;

  // Create subscription to cmd_vel
  if (rclc_subscription_init_default(
        &subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel") != RCL_RET_OK) return false;

  // Create executor with 1 handle (subscription only, PID runs in loop)
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
  Serial.println("\n========================================");
  Serial.println(" ESP32 PID Robot - micro-ROS Controller");
  Serial.println("========================================");

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

  // Encoder pins
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), enc_L_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), enc_R_ISR, RISING);

  // micro-ROS WiFi transport
  set_microros_wifi_transports((char*)ssid, (char*)password,
                                (char*)agent_ip, agent_port);

  Serial.printf("[WiFi] Connecting to %s ...\n", ssid);
  Serial.printf("[uROS] Agent: %s:%d\n", agent_ip, agent_port);

  stop_motors();
  last_pid_time   = millis();
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
          target_speed_L = 0.0;
          target_speed_R = 0.0;
        }

        // PID update at fixed interval
        if ((now - last_pid_time) >= (unsigned long)(DT * 1000)) {
          last_pid_time = now;
          pid_update();
        }

        // Debug prints every 500ms
        if ((now - last_debug_time) >= 500) {
          last_debug_time = now;
          Serial.printf("[DBG] tgt_L=%.3f tgt_R=%.3f | cur_L=%.3f cur_R=%.3f | ticks L=%ld R=%ld\n",
                        target_speed_L, target_speed_R,
                        current_speed_L, current_speed_R,
                        ticks_L, ticks_R);
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
