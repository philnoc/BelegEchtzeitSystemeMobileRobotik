#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

// Initialisiere die Motor-Serielle Schnittstelle
void initMotorSerial(int rxPin, int txPin, long baudrate);

// Sende Befehle an das Motorboard
void sendeMotorBefehl(uint8_t id, uint8_t modus, uint8_t richtung, uint16_t schritte);

// Stoppe beide Motoren
void stoppeMotoren();

// Führt Bewegungslogik aus
void motorControl(float linear, float angular);

#endif
