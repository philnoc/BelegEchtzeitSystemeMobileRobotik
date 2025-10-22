#include "motor.h"

HardwareSerial MotorSerial(2); // UART2 für Motor

void initMotorSerial(int rxPin, int txPin, long baudrate) {
    MotorSerial.begin(baudrate, SERIAL_8N1, rxPin, txPin);
}

void sendeMotorBefehl(uint8_t id, uint8_t modus, uint8_t richtung, uint16_t schritte) {
    uint8_t motor_cmd[] = {
        0xAA,
        id,
        modus,
        richtung,
        (uint8_t)((schritte >> 8) & 0xFF),
        (uint8_t)(schritte & 0xFF)
    };
    MotorSerial.write(motor_cmd, sizeof(motor_cmd));
}

void stoppeMotoren() {
    sendeMotorBefehl(0x01, 0x01, 0x00, 0);
    sendeMotorBefehl(0x02, 0x01, 0x01, 0);
}

void motor1Control(float linear, float angular) {
     // Richtungslogik bleibt unverändert
  if (linear >= 2.0 && angular==0.0) {
    sendeMotorBefehl(0x01, 0x01, 0x01, 5);
    sendeMotorBefehl(0x02, 0x01, 0x00, 5);
  } else if (linear >= 1.0 && angular==0.0){
    sendeMotorBefehl(0x01, 0x01, 0x01, 10);
    sendeMotorBefehl(0x02, 0x01, 0x00, 10);
  } else if (linear >= 0.5 && angular==0.0){
    sendeMotorBefehl(0x01, 0x01, 0x01, 20);
    sendeMotorBefehl(0x02, 0x01, 0x00, 20);
  } else if (linear <= -2.0 && angular==0.0) {
    sendeMotorBefehl(0x01, 0x01, 0x00, 5);
    sendeMotorBefehl(0x02, 0x01, 0x01, 5);
  } else if (linear <= -1.0 && angular==0.0){
    sendeMotorBefehl(0x01, 0x01, 0x00, 10);
    sendeMotorBefehl(0x02, 0x01, 0x01, 10);
  } else if (linear <= -0.5 && angular==0.0){
    sendeMotorBefehl(0x01, 0x01, 0x00, 20);
    sendeMotorBefehl(0x02, 0x01, 0x01, 20);
  } else if (angular >= 2.0 && linear==0.0){
    sendeMotorBefehl(0x01, 0x01, 0x00, 10);
    sendeMotorBefehl(0x02, 0x01, 0x00, 10);
  } else if (angular >= 1.0 && linear==0.0){
    sendeMotorBefehl(0x01, 0x01, 0x00, 20);
    sendeMotorBefehl(0x02, 0x01, 0x00, 20);
  } else if (angular <= -2.0 && linear==0.0){
    sendeMotorBefehl(0x01, 0x01, 0x01, 10);
    sendeMotorBefehl(0x02, 0x01, 0x01, 10);
  } else if (angular <= -1.0 && linear==0.0){
    sendeMotorBefehl(0x01, 0x01, 0x01, 20);
    sendeMotorBefehl(0x02, 0x01, 0x01, 20);
  } else if (angular >= 0.5 && linear==0.0){
    sendeMotorBefehl(0x01, 0x01, 0x00, 40);
    sendeMotorBefehl(0x02, 0x01, 0x00, 40);
  } else if (angular >= 0.1 && linear==0.0){
    sendeMotorBefehl(0x01, 0x01, 0x00, 80);
    sendeMotorBefehl(0x02, 0x01, 0x00, 80);
  } else if (angular <= -0.5 && linear==0.0){
    sendeMotorBefehl(0x01, 0x01, 0x01, 40);
    sendeMotorBefehl(0x02, 0x01, 0x01, 40);
  } else if (angular <= -0.1 && linear==0.0){
    sendeMotorBefehl(0x01, 0x01, 0x01, 80);
    sendeMotorBefehl(0x02, 0x01, 0x01, 80);
  } else if (linear >= 2.0 && angular>=2.0){ //vorwärts und links
    sendeMotorBefehl(0x01, 0x01, 0x01, 20); //
    sendeMotorBefehl(0x02, 0x01, 0x00, 5);
  } else if (linear >= 1.0 && angular>=1.0){ //vorwärts und links
    sendeMotorBefehl(0x01, 0x01, 0x01, 40); //
    sendeMotorBefehl(0x02, 0x01, 0x00, 10);
  } else if (linear >= 0.5 && angular>=0.5){ //vorwärts und links
    sendeMotorBefehl(0x01, 0x01, 0x01, 80); //
    sendeMotorBefehl(0x02, 0x01, 0x00, 20);
  } else if (linear >= 0.1 && angular>=0.1){ //vorwärts und links
    sendeMotorBefehl(0x01, 0x01, 0x01, 160); //
    sendeMotorBefehl(0x02, 0x01, 0x00, 40);
  } else if (linear <= -2.0 && angular>=2.0){ //rückwärts und links
    sendeMotorBefehl(0x01, 0x01, 0x00, 20); //
    sendeMotorBefehl(0x02, 0x01, 0x01, 5);
  } else if (linear <= -1.0 && angular>=1.0){ //rückwärts und links
    sendeMotorBefehl(0x01, 0x01, 0x00, 40); //
    sendeMotorBefehl(0x02, 0x01, 0x01, 10);
  } else if (linear <= -0.5 && angular>=0.5){ //rückwärts und links
    sendeMotorBefehl(0x01, 0x01, 0x00, 80); //
    sendeMotorBefehl(0x02, 0x01, 0x01, 20);
  } else if (linear <= -0.1 && angular>=0.1){ //rückwärts und links
    sendeMotorBefehl(0x01, 0x01, 0x00, 160); //
    sendeMotorBefehl(0x02, 0x01, 0x01, 40);
  } else if (linear >= 2.0 && angular<=-2.0){ //vorwärts und rechts
    sendeMotorBefehl(0x01, 0x01, 0x01, 5); //
    sendeMotorBefehl(0x02, 0x01, 0x00, 20);
  } else if (linear >= 1.0 && angular<=-1.0){ //vorwärts und rechts
    sendeMotorBefehl(0x01, 0x01, 0x01, 10); //
    sendeMotorBefehl(0x02, 0x01, 0x00, 40);
  } else if (linear >= 0.5 && angular<=-0.5){ //vorwärts und rechts
    sendeMotorBefehl(0x01, 0x01, 0x01, 20); //
    sendeMotorBefehl(0x02, 0x01, 0x00, 80);
  } else if (linear >= 0.1 && angular<=-0.1){ //vorwärts und rechts
    sendeMotorBefehl(0x01, 0x01, 0x01, 40); //
    sendeMotorBefehl(0x02, 0x01, 0x00, 160);
  } else if (linear <= -2.0 && angular<=-2.0){ //rückwärts und rechts
    sendeMotorBefehl(0x01, 0x01, 0x00, 5); //
    sendeMotorBefehl(0x02, 0x01, 0x01, 20);
  } else if (linear <= -1.0 && angular<=-1.0){ //rückwärts und rechts
    sendeMotorBefehl(0x01, 0x01, 0x00, 10); //
    sendeMotorBefehl(0x02, 0x01, 0x01, 40);
  } else if (linear <= -0.5 && angular<=-0.5){ //rückwärts und rechts
    sendeMotorBefehl(0x01, 0x01, 0x00, 20); //
    sendeMotorBefehl(0x02, 0x01, 0x01, 80);
  } else if (linear <= -0.1 && angular<=-0.1){ //rückwärts und rechts
    sendeMotorBefehl(0x01, 0x01, 0x00, 40); //
    sendeMotorBefehl(0x02, 0x01, 0x01, 160);
  }else {
    stoppeMotoren();
  }
}

void motorControl(float linear, float angular) {
    // Differenzial-Antrieb: linke und rechte "Geschwindigkeit"
    float left  = linear - angular;
    float right = linear + angular;

    // Mapping: steps ~ 1/v
    auto mapSpeed = [](float v) -> uint16_t {
    v = fabs(v);
    if (v < 0.05) return 0;   // Totzone

    float k = 5.0;            // bei v=1 -> 5 Steps
    uint16_t steps = (uint16_t)(k / v);

    if (steps < 2) steps = 2; // Sicherheitsgrenze für Motor
    return steps;
};

    // Richtung setzen
    uint8_t leftDir  = (left  >= 0) ? 0x01 : 0x00;
    uint8_t rightDir = (right >= 0) ? 0x00 : 0x01;  // beide gleich -> symmetrisch

    // Schritte berechnen
    uint16_t leftSteps  = mapSpeed(left);
    uint16_t rightSteps = mapSpeed(right);

    if (leftSteps == 0 && rightSteps == 0) {
        stoppeMotoren();
    } else {
        sendeMotorBefehl(0x01, 0x01, leftDir,  leftSteps);
        sendeMotorBefehl(0x02, 0x01, rightDir, rightSteps);
    }
}
