#include <WiFi.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// ==========================================
// WIFI
// ==========================================

char ssid[] = "OnLineHS";
char password[] = "0202-totines";

char agent_ip[] = "10.0.1.154";
const uint16_t agent_port = 8888;

// ==========================================
// PINES DEL HARDWARE
// ==========================================

const int STBY = 26;

const int PWMA = 25;
const int AIN1 = 32;
const int AIN2 = 33;

const int PWMB = 13;
const int BIN1 = 14;
const int BIN2 = 12;

const int ENC_L_A = 34;
const int ENC_L_B = 35;

const int ENC_R_A = 23;
const int ENC_R_B = 19;

// ==========================================
// PARÁMETROS DEL ROBOT
// ==========================================

const float WHEEL_BASE = 0.20;
const float WHEEL_RADIUS = 0.02;

const int TICKS_PER_REV = 150; // Calibrado

// ==========================================
// PID
// ==========================================

float Kp = 0.5;
float Ki = 0.0;
float Kd = 0.01;

const unsigned long PID_INTERVAL = 50;
const float DT = 0.05;
const float INTEGRAL_LIMIT = 50.0;

// ==========================================
// SEGURIDAD
// ==========================================

unsigned long last_cmd_time = 0;
const unsigned long CMD_TIMEOUT = 500;

// ==========================================
// micro-ROS
// ==========================================

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

bool micro_ros_connected = false;
int spin_fail_count = 0;

// ==========================================
// ENCODERS
// ==========================================

volatile long ticks_L = 0;
volatile long ticks_R = 0;

long prev_ticks_L = 0;
long prev_ticks_R = 0;

// ==========================================
// REFERENCIAS
// ==========================================

volatile float target_speed_L = 0.0;
volatile float target_speed_R = 0.0;

// ==========================================
// PID VARIABLES
// ==========================================

float integral_L = 0.0;
float prev_error_L = 0.0;

float integral_R = 0.0;
float prev_error_R = 0.0;

// ==========================================
// FILTRO VELOCIDAD
// ==========================================

float filtered_speed_L = 0.0;
float filtered_speed_R = 0.0;
const float FILTER_ALPHA = 0.15;

// ==========================================
// TIMING
// ==========================================

unsigned long last_pid_time = 0;

// ==========================================
// ENCODERS ISR
// ==========================================

void IRAM_ATTR countLeft()
{
  if (digitalRead(ENC_L_B))
    ticks_L++;
  else
    ticks_L--;
}

void IRAM_ATTR countRight()
{
  if (digitalRead(ENC_R_B))
    ticks_R--;
  else
    ticks_R++;
}

// ==========================================
// CMD_VEL CALLBACK
// ==========================================

void twist_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * twist_msg =
      (const geometry_msgs__msg__Twist *)msgin;

  float v = twist_msg->linear.x;
  float w = twist_msg->angular.z;

  target_speed_L = v - (w * WHEEL_BASE / 2.0);
  target_speed_R = v + (w * WHEEL_BASE / 2.0);

  last_cmd_time = millis();

  Serial.print(">> CMD_VEL: v=");
  Serial.print(v);
  Serial.print(" w=");
  Serial.println(w);
}

// ==========================================
// MOTOR
// ==========================================

void actuate_motor(
    float control_signal,
    int pin_in1,
    int pin_in2,
    int pin_pwm)
{
  if (abs(control_signal) < 0.01)
  {
    digitalWrite(pin_in1, LOW);
    digitalWrite(pin_in2, LOW);
    analogWrite(pin_pwm, 0);
    return;
  }

  int pwm_val =
      constrain(
          (int)(abs(control_signal) * 200.0),
          60,
          200);

  if (control_signal > 0)
  {
    digitalWrite(pin_in1, HIGH);
    digitalWrite(pin_in2, LOW);
  }
  else
  {
    digitalWrite(pin_in1, LOW);
    digitalWrite(pin_in2, HIGH);
  }

  analogWrite(pin_pwm, pwm_val);
}

// ==========================================
// DETENER MOTORES
// ==========================================

void stop_motors()
{
  target_speed_L = 0.0;
  target_speed_R = 0.0;
  integral_L = 0.0;
  integral_R = 0.0;
  prev_error_L = 0.0;
  prev_error_R = 0.0;
  filtered_speed_L = 0.0;
  filtered_speed_R = 0.0;

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 0);
}

// ==========================================
// PID LOOP (llamado desde loop, NO del executor)
// ==========================================

void run_pid()
{
  // Timeout de seguridad
  if (millis() - last_cmd_time > CMD_TIMEOUT)
  {
    target_speed_L = 0.0;
    target_speed_R = 0.0;
  }

  // Leer ticks actuales
  long current_ticks_L = ticks_L;
  long current_ticks_R = ticks_R;

  // Calcular velocidad cruda (m/s)
  float raw_speed_L =
      ((current_ticks_L - prev_ticks_L)
      * 2.0 * PI * WHEEL_RADIUS)
      / (TICKS_PER_REV * DT);

  float raw_speed_R =
      ((current_ticks_R - prev_ticks_R)
      * 2.0 * PI * WHEEL_RADIUS)
      / (TICKS_PER_REV * DT);

  prev_ticks_L = current_ticks_L;
  prev_ticks_R = current_ticks_R;

  // Filtro pasa-bajos
  filtered_speed_L =
      FILTER_ALPHA * raw_speed_L
      + (1.0 - FILTER_ALPHA) * filtered_speed_L;

  filtered_speed_R =
      FILTER_ALPHA * raw_speed_R
      + (1.0 - FILTER_ALPHA) * filtered_speed_R;

  // Copiar targets (pueden cambiar desde callback)
  float ref_L = target_speed_L;
  float ref_R = target_speed_R;

  // PID izquierdo
  float error_L = ref_L - filtered_speed_L;
  integral_L += error_L * DT;
  integral_L = constrain(integral_L, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float derivative_L = (error_L - prev_error_L) / DT;
  float control_L = (Kp * error_L) + (Ki * integral_L) + (Kd * derivative_L);
  prev_error_L = error_L;

  // PID derecho
  float error_R = ref_R - filtered_speed_R;
  integral_R += error_R * DT;
  integral_R = constrain(integral_R, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float derivative_R = (error_R - prev_error_R) / DT;
  float control_R = (Kp * error_R) + (Ki * integral_R) + (Kd * derivative_R);
  prev_error_R = error_R;

  // Actuar motores
  actuate_motor(control_L, AIN1, AIN2, PWMA);
  actuate_motor(control_R, BIN1, BIN2, PWMB);

  // Debug
  Serial.print("RefL=");
  Serial.print(ref_L, 2);
  Serial.print(" VelL=");
  Serial.print(filtered_speed_L, 2);
  Serial.print(" RefR=");
  Serial.print(ref_R, 2);
  Serial.print(" VelR=");
  Serial.println(filtered_speed_R, 2);
}

// ==========================================
// CREAR ENTIDADES micro-ROS (sin timer)
// ==========================================

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  Serial.println(">> Creando support...");
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
  {
    Serial.println("   FALLO");
    return false;
  }
  Serial.println("   OK");
  delay(500);

  Serial.println(">> Creando nodo...");
  if (rclc_node_init_default(&node, "cmd_vel_subscriber_pid", "", &support) != RCL_RET_OK)
  {
    Serial.println("   FALLO");
    return false;
  }
  Serial.println("   OK");
  delay(500);

  Serial.println(">> Creando suscripcion...");
  if (rclc_subscription_init_default(
          &subscriber,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
          "cmd_vel") != RCL_RET_OK)
  {
    Serial.println("   FALLO");
    return false;
  }
  Serial.println("   OK");
  delay(500);

  // Executor solo con 1 handle: la suscripcion
  Serial.println(">> Creando executor...");
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK)
  {
    Serial.println("   FALLO");
    return false;
  }
  Serial.println("   OK");
  delay(200);

  Serial.println(">> Agregando suscripcion...");
  if (rclc_executor_add_subscription(&executor, &subscriber, &msg, &twist_callback, ON_NEW_DATA) != RCL_RET_OK)
  {
    Serial.println("   FALLO");
    return false;
  }
  Serial.println("   OK");

  return true;
}

// ==========================================
// DESTRUIR ENTIDADES micro-ROS
// ==========================================

void destroy_entities()
{
  Serial.println(">> Destruyendo entidades...");
  rcl_subscription_fini(&subscriber, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  Serial.println("   Listo");
}

// ==========================================
// SETUP
// ==========================================

void setup()
{
  Serial.begin(115200);
  delay(2000);

  // ---- WiFi ----

  set_microros_wifi_transports(
      ssid,
      password,
      agent_ip,
      agent_port);

  Serial.print("Conectando WiFi");

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi conectado");
  Serial.print("IP ESP32: ");
  Serial.println(WiFi.localIP());

  // ---- Pines ----

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(ENC_L_A, INPUT);
  pinMode(ENC_L_B, INPUT);

  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(
      digitalPinToInterrupt(ENC_L_A),
      countLeft,
      RISING);

  attachInterrupt(
      digitalPinToInterrupt(ENC_R_A),
      countRight,
      RISING);

  // ---- Estado inicial ----

  micro_ros_connected = false;
  spin_fail_count = 0;
  last_pid_time = millis();

  Serial.println("===========================");
  Serial.println("Conectando a micro-ROS...");
  Serial.println("===========================");
}

// ==========================================
// LOOP
// ==========================================

void loop()
{
  // ---- CONEXIÓN micro-ROS ----

  if (!micro_ros_connected)
  {
    Serial.println(">> Intentando conectar...");

    if (create_entities())
    {
      Serial.println("===========================");
      Serial.println("CONECTADO - Robot listo!");
      Serial.println("===========================");
      last_cmd_time = millis();
      last_pid_time = millis();
      micro_ros_connected = true;
      spin_fail_count = 0;
    }
    else
    {
      Serial.println(">> Fallo, reintentando en 2s...");
      destroy_entities();
      stop_motors();
      delay(2000);
    }
    return;
  }

  // ---- PROCESAR SUSCRIPCIÓN (no bloqueante) ----

  rcl_ret_t rc = rclc_executor_spin_some(
      &executor,
      RCL_MS_TO_NS(10));

  if (rc != RCL_RET_OK)
  {
    spin_fail_count++;
    if (spin_fail_count > 100)
    {
      Serial.println(">> Desconexion detectada");
      destroy_entities();
      stop_motors();
      micro_ros_connected = false;
      spin_fail_count = 0;
      delay(2000);
      return;
    }
  }
  else
  {
    spin_fail_count = 0;
  }

  // ---- PID cada 50ms (independiente del executor) ----

  if (millis() - last_pid_time >= PID_INTERVAL)
  {
    last_pid_time = millis();
    run_pid();
  }
}
