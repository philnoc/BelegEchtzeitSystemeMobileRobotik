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
    uint8_t rightDir = (right >= 0) ? 0x00 : 0x01; 

    // Schritte berechnen
    uint16_t leftSteps  = mapSpeed(left);
    uint16_t rightSteps = mapSpeed(right);

    sendeMotorBefehl(0x01, 0x01, leftDir,  leftSteps);
    sendeMotorBefehl(0x02, 0x01, rightDir, rightSteps);
}
