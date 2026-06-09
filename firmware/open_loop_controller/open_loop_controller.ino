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

// ==========================================
// PARÁMETROS DEL ROBOT
// ==========================================

const float WHEEL_BASE = 0.20;

// Ajusta este valor para cambiar velocidad máxima
// 150 = moderado, 200 = rápido, 100 = lento
const int MAX_PWM = 150;

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
// VELOCIDADES
// ==========================================

volatile float cmd_linear = 0.0;
volatile float cmd_angular = 0.0;

// ==========================================
// CMD_VEL CALLBACK
// ==========================================

void twist_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * twist_msg =
      (const geometry_msgs__msg__Twist *)msgin;

  cmd_linear = twist_msg->linear.x;
  cmd_angular = twist_msg->angular.z;

  last_cmd_time = millis();

  Serial.print(">> CMD_VEL: v=");
  Serial.print(cmd_linear);
  Serial.print(" w=");
  Serial.println(cmd_angular);
}

// ==========================================
// MOTOR (open-loop directo)
// ==========================================

void set_motor(
    int pwm_val,
    int pin_in1,
    int pin_in2,
    int pin_pwm)
{
  if (pwm_val > 0)
  {
    digitalWrite(pin_in1, HIGH);
    digitalWrite(pin_in2, LOW);
    analogWrite(pin_pwm, pwm_val);
  }
  else if (pwm_val < 0)
  {
    digitalWrite(pin_in1, LOW);
    digitalWrite(pin_in2, HIGH);
    analogWrite(pin_pwm, -pwm_val);
  }
  else
  {
    digitalWrite(pin_in1, LOW);
    digitalWrite(pin_in2, LOW);
    analogWrite(pin_pwm, 0);
  }
}

// ==========================================
// DETENER MOTORES
// ==========================================

void stop_motors()
{
  cmd_linear = 0.0;
  cmd_angular = 0.0;

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 0);
}

// ==========================================
// CONTROL DE MOTORES
// ==========================================

void update_motors()
{
  // Timeout de seguridad
  if (millis() - last_cmd_time > CMD_TIMEOUT)
  {
    cmd_linear = 0.0;
    cmd_angular = 0.0;
  }

  // Mapear directo: linear = ambas ruedas, angular = diferencial
  int base_pwm = constrain((int)(cmd_linear * MAX_PWM), -MAX_PWM, MAX_PWM);
  int turn_pwm = constrain((int)(cmd_angular * MAX_PWM), -MAX_PWM, MAX_PWM);

  int pwm_L = constrain(base_pwm - turn_pwm, -MAX_PWM, MAX_PWM);
  int pwm_R = constrain(base_pwm + turn_pwm, -MAX_PWM, MAX_PWM);

  set_motor(pwm_L, AIN1, AIN2, PWMA);
  set_motor(pwm_R, BIN1, BIN2, PWMB);

  Serial.print("PWM_L=");
  Serial.print(pwm_L);
  Serial.print(" PWM_R=");
  Serial.println(pwm_R);
}

// ==========================================
// CREAR ENTIDADES micro-ROS
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
  if (rclc_node_init_default(&node, "cmd_vel_subscriber", "", &support) != RCL_RET_OK)
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

  // ---- Estado inicial ----

  micro_ros_connected = false;
  spin_fail_count = 0;

  Serial.println("===========================");
  Serial.println("Conectando a micro-ROS...");
  Serial.println("===========================");
}

// ==========================================
// LOOP
// ==========================================

void loop()
{
  // ---- CONEXIÓN ----

  if (!micro_ros_connected)
  {
    Serial.println(">> Intentando conectar...");

    if (create_entities())
    {
      Serial.println("===========================");
      Serial.println("CONECTADO - Robot listo!");
      Serial.println("===========================");
      last_cmd_time = millis();
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

  // ---- PROCESAR micro-ROS ----

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

  // ---- ACTUALIZAR MOTORES cada 50ms ----

  static unsigned long last_motor_time = 0;
  if (millis() - last_motor_time >= 50)
  {
    last_motor_time = millis();
    update_motors();
  }
}
