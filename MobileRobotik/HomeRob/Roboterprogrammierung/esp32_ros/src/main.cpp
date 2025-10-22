#include <HardwareSerial.h>
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <Wire.h>
#include "motor.h"
#include "display.h"
#include "sound.h"
#include "ultrasonic.h"
#include "imu.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/bool.h>
#include <sensor_msgs/msg/imu.h>

#define UART_TX_PIN 17
#define UART_RX_PIN -1
#define UART_BAUDRATE 9600
#define BumperL 12
#define BumperR 14

// ROS Objekte
rcl_subscription_t subscriber_cmd_vel;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
geometry_msgs__msg__Twist twist_msg;
rcl_publisher_t range_publisher;
rcl_publisher_t bumper1_publ;
rcl_publisher_t bumper2_publ;
rcl_publisher_t imu_publisher;
sensor_msgs__msg__Range range_msg;
rcl_subscription_t subscriber_display;
std_msgs__msg__String string_msg_sound;
std_msgs__msg__String string_msg_display;
rcl_subscription_t subscriber_sound;
std_msgs__msg__Bool bool_bl_msg;
std_msgs__msg__Bool bool_br_msg;
sensor_msgs__msg__Imu imu_msg;

// Timeout-Logik
unsigned long last_cmd_time = 0;
const unsigned long CMD_TIMEOUT_MS = 500; // z. B. 500ms ohne neue Nachricht = Stop

// Variablen für IMU
xyzFloat gValue;
xyzFloat gyr;
xyzFloat magValue;
float tempValue;


// Fehlerprüfung
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();} }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} }

void error_loop() {
  while(1) { delay(100); }
}

//Subscriber Callback Sound
void sound_callback(const void *msgin) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
  String Sound = msg->data.data;

  if (Sound == "Supermario") {
    SuperMario();
  } else if (Sound == "IndianaJones") {
    IndianaJones();
  } else {
    playCustomMelody(Sound.c_str());  // Neue dynamische Variante
  }
}

//Subscriber Callback Display
void display_string_callback(const void *msgin) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
  showMessage(msg->data.data);
}

// Subscriber Callback Motor
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  float linear = msg->linear.x;
  float angular = msg->angular.z;
  last_cmd_time = millis();  // Zeitstempel aktualisieren
  motorControl(linear, angular);
}

void setup() {
  Wire.begin();
  Serial.begin(115200); //Serielle Verbindung zum PC
  set_microros_serial_transports(Serial); //Ros Kommunikation über seriell
  initUltrasonic(range_msg);
  pinMode(BumperL, INPUT);
  pinMode(BumperR, INPUT);
  initDisplay(string_msg_display);
  init_IMU();
  initSound(string_msg_sound);
  delay(2000);

  initMotorSerial(UART_RX_PIN, UART_TX_PIN, UART_BAUDRATE);

  allocator = rcl_get_default_allocator(); //Ros CLient Library Initialisierung
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); //Initialisierung Ros SUpport Struktur
  RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));//Initialisierung Ros2 Node mit namen esp32_cmd_vel_node

  RCCHECK(rclc_subscription_init_default(
    &subscriber_display,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "display")); //

  RCCHECK(rclc_subscription_init_default(
    &subscriber_sound,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "sound")); //

  RCCHECK(rclc_subscription_init_default(
    &subscriber_cmd_vel,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel")); //ESP hört auf das Topic cmd_vel

  RCCHECK(rclc_publisher_init_default(
  &range_publisher,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
  "ultrasonic_range"));

  RCCHECK(rclc_publisher_init_default(
  &bumper1_publ,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
  "LeftBumper"));

  RCCHECK(rclc_publisher_init_default(
  &bumper2_publ,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
  "RightBumper"));

  RCCHECK(rclc_publisher_init_default(
  &imu_publisher,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
  "imu_values"));


  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  // subscriber_cmd_vel (für cmd_vel)
RCCHECK(rclc_executor_add_subscription(
  &executor,
  &subscriber_cmd_vel,
  &twist_msg,
  &cmd_vel_callback,
  ON_NEW_DATA));

// subscriber_display (für display String)
RCCHECK(rclc_executor_add_subscription(
  &executor,
  &subscriber_display,
  &string_msg_display,
  &display_string_callback,
  ON_NEW_DATA));

RCCHECK(rclc_executor_add_subscription(
  &executor,
  &subscriber_sound,
  &string_msg_sound,
  &sound_callback,
  ON_NEW_DATA));
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  
  // Frequenzbegrenzung: Nur alle 200 ms senden
  static unsigned long last_pub_time = 0;
  unsigned long now = millis();

  if (now - last_pub_time >= 200) {  // alle 200 ms publizieren
    last_pub_time = now;

    long dist = readDistanceCm();
    range_msg.header.stamp.sec = now / 1000;
    range_msg.header.stamp.nanosec = (now % 1000) * 1000000;
    range_msg.range = (dist > 0) ? dist / 100.0 : 0.0;

    bool_bl_msg.data=!digitalRead(BumperL);
    bool_br_msg.data=!digitalRead(BumperR);

    imu_msg.header.stamp.sec=now/1000;
    imu_msg.header.stamp.nanosec = (now % 1000) * 1000000;
    getValuesIMU(&gValue, &gyr, &magValue, &tempValue);
    imu_msg.angular_velocity.x = gyr.x * M_PI / 180.0f;
    imu_msg.angular_velocity.y = gyr.y * M_PI / 180.0f;
    imu_msg.angular_velocity.z = gyr.z * M_PI / 180.0f;

    imu_msg.linear_acceleration.x=gValue.x*9.8f;
    imu_msg.linear_acceleration.y=gValue.y *9.8f;
    imu_msg.linear_acceleration.z=gValue.z*9.8f;

    rcl_publish(&range_publisher, &range_msg, NULL);
    rcl_publish(&bumper1_publ, &bool_bl_msg, NULL);
    rcl_publish(&bumper2_publ, &bool_br_msg, NULL);
    rcl_publish(&imu_publisher, &imu_msg, NULL);
  }
  // Timeout-Überprüfung für Stop
  if (now - last_cmd_time > CMD_TIMEOUT_MS) {
    stoppeMotoren();
  }
  delay(10);  // optional, zur Entlastung der CPU
}