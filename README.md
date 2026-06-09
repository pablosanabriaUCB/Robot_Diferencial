# Robot Diferencial con micro-ROS y ESP32

Control de un robot diferencial mediante **micro-ROS** sobre **WiFi (UDP)**, utilizando un **ESP32 de 30 pines** y un driver de motores **TB6612FNG**.

## 📁 Estructura del Repositorio

```
├── firmware/
│   ├── micro_ros_pid_controller/    # Controlador PID con encoders
│   │   └── micro_ros_pid_controller.ino
│   └── open_loop_controller/        # Control open-loop (sin PID)
│       └── open_loop_controller.ino
├── ros2/
│   ├── launch/
│   │   └── micro_ros_agent.launch.py
│   ├── config/
│   │   └── agent_params.yaml
│   └── scripts/
│       └── run_agent.sh
└── README.md
```

## 🔧 Hardware

| Componente | Descripción |
|---|---|
| Microcontrolador | ESP32 DevKit v1 (30 pines) |
| Driver de motores | TB6612FNG |
| Motores | DC con encoders de cuadratura |
| Comunicación | WiFi UDP (micro-ROS) |

### Pinout ESP32

| Función | Pin |
|---|---|
| STBY (driver enable) | GPIO 26 |
| Motor A - PWM | GPIO 25 |
| Motor A - IN1 | GPIO 32 |
| Motor A - IN2 | GPIO 33 |
| Motor B - PWM | GPIO 13 |
| Motor B - IN1 | GPIO 14 |
| Motor B - IN2 | GPIO 12 |
| Encoder L - Canal A | GPIO 34 |
| Encoder L - Canal B | GPIO 35 |
| Encoder R - Canal A | GPIO 23 |
| Encoder R - Canal B | GPIO 19 |

### Parámetros del Robot

| Parámetro | Valor |
|---|---|
| Distancia entre ruedas (WHEEL_BASE) | 0.20 m |
| Radio de rueda (WHEEL_RADIUS) | 0.02 m |
| Ticks por revolución | 330 |

## 🛠️ Requisitos Previos

### PC (Ubuntu con ROS 2 Jazzy)

```bash
# ROS 2 Jazzy debe estar instalado y sourceado
source /opt/ros/jazzy/setup.bash

# Docker debe estar instalado
docker --version

# Descargar imagen del agente micro-ROS
sudo docker pull microros/micro-ros-agent:jazzy
```

### Arduino IDE

1. Instalar **Arduino IDE** (v2.x recomendado)
2. Agregar el board ESP32:
   - Ir a `File > Preferences > Additional Board Manager URLs`
   - Agregar: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
   - Ir a `Tools > Board > Board Manager` e instalar **esp32 by Espressif**
3. Instalar la librería **micro_ros_arduino**:
   - Descargar la versión `2.0.8-jazzy` desde [GitHub](https://github.com/micro-ROS/micro_ros_arduino/releases)
   - Ir a `Sketch > Include Library > Add .ZIP Library` y seleccionar el archivo descargado

## 📦 Compilación y Flash

### 1. Configurar WiFi y IP del agente

Editar las siguientes líneas en el archivo `.ino` que vayas a usar:

```c
char ssid[] = "TU_SSID";           // Nombre de tu red WiFi
char password[] = "TU_PASSWORD";    // Contraseña WiFi
char agent_ip[] = "10.0.1.154";     // IP de tu PC (donde corre el agente)
const uint16_t agent_port = 8888;   // Puerto UDP
```

Para obtener la IP de tu PC:

```bash
hostname -I
```

### 2. Compilar y flashear

1. Abrir el archivo `.ino` en Arduino IDE
2. Seleccionar board: `Tools > Board > ESP32 Dev Module`
3. Seleccionar puerto: `Tools > Port > /dev/ttyUSB0` (o el que corresponda)
4. Compilar y subir: `Sketch > Upload` (o Ctrl+U)
5. Abrir el Monitor Serial para verificar: `Tools > Serial Monitor` (115200 baud)

El Monitor Serial debería mostrar:

```
Conectando WiFi
WiFi conectado
IP ESP32: 10.0.1.X
===========================
Conectando a micro-ROS...
===========================
>> Intentando conectar...
>> Creando support...
   OK
>> Creando nodo...
   OK
>> Creando suscripcion...
   OK
...
===========================
CONECTADO - Robot listo!
===========================
```

## 🚀 Ejecución

### Paso 1: Iniciar el agente micro-ROS (en tu PC)

Opción A — Con el script:

```bash
cd ros2/scripts
chmod +x run_agent.sh
./run_agent.sh
```

Opción B — Directamente con Docker:

```bash
sudo docker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888 -v6
```

### Paso 2: Encender el ESP32

Alimentar el ESP32 con batería o fuente externa. Se conectará automáticamente por WiFi al agente.

### Paso 3: Verificar conexión

En otra terminal:

```bash
# Ver nodo activo
ros2 node list
# Debería mostrar: /cmd_vel_subscriber_pid  (o /cmd_vel_subscriber)

# Ver topics
ros2 topic list
# Debería mostrar: /cmd_vel
```

### Paso 4: Enviar comandos de movimiento

```bash
# Avanzar
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Retroceder
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: -0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Girar sobre su eje (izquierda)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# Girar sobre su eje (derecha)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}"

# Frenar
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

> **Nota:** El robot frena automáticamente si deja de recibir comandos después de 500ms (CMD_TIMEOUT).

## 🎛️ Valores Finales del PID

| Parámetro | Valor | Descripción |
|---|---|---|
| **Kp** | 0.5 | Ganancia proporcional |
| **Ki** | 0.0 | Ganancia integral (desactivada para evitar wind-up) |
| **Kd** | 0.01 | Ganancia derivativa |
| **DT** | 0.05 s | Periodo de muestreo (50 ms) |
| **INTEGRAL_LIMIT** | 50.0 | Límite anti-windup del integrador |
| **CMD_TIMEOUT** | 500 ms | Timeout de seguridad (frena si no recibe comandos) |
| **FILTER_ALPHA** | 0.15 | Coeficiente del filtro pasa-bajos en la velocidad |
| **PWM mínimo** | 60 | PWM mínimo para vencer zona muerta del motor |
| **PWM máximo** | 200 | PWM máximo aplicable |

### Proceso de sintonización

1. Se inició con valores agresivos (Kp=1.5, Ki=0.1, Kd=0.05) que causaban oscilaciones y velocidades fuera de control.
2. Se redujo Kp a 0.5 para una respuesta más suave.
3. Se desactivó Ki (=0.0) para eliminar el efecto de wind-up integral que hacía que los motores aceleraran sin control.
4. Se redujo Kd a 0.01 para minimizar la amplificación del ruido de los encoders.
5. Se agregó un filtro pasa-bajos (alpha=0.15) sobre la velocidad medida para suavizar el ruido eléctrico de los encoders.
6. Se implementó compensación de zona muerta con PWM mínimo de 60 para garantizar que los motores arranquen.

## 📡 Arquitectura de Comunicación

```
┌──────────────┐     WiFi UDP      ┌──────────────────┐     DDS      ┌──────────┐
│   ESP32      │◄──────────────────►│  micro-ROS Agent │◄────────────►│  ROS 2   │
│  (firmware)  │    puerto 8888     │    (Docker)      │              │  topics  │
└──────────────┘                    └──────────────────┘              └──────────┘
```

- El ESP32 se suscribe al topic `/cmd_vel` (`geometry_msgs/msg/Twist`)
- El agente micro-ROS (Docker) actúa como puente entre el ESP32 y la red DDS de ROS 2
- El flag `--net=host` en Docker permite que el agente comparta la red del host

## ⚠️ Notas Importantes

- **El ESP32 debe reiniciarse después de reiniciar el agente** para que las entidades se creen correctamente.
- Si la creación de entidades falla (común por pérdida de paquetes UDP en WiFi), el firmware reintenta automáticamente cada 2 segundos.
- Los pins GPIO 34 y 35 del ESP32 son solo entrada y **no soportan pull-up interno**. Si se usan para encoders, se requieren resistencias pull-up externas.
- El control open-loop (`open_loop_controller/`) no usa encoders ni PID — mapea `cmd_vel` directamente a PWM. Útil para pruebas rápidas.
