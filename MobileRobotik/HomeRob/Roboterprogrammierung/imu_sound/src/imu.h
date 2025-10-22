#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <MPU9250_WE.h>
#include <Wire.h>
const uint8_t MPU9250_ADDR = 0x68;   // evtl. auf 0x69 ändern je nach Verdrahtung

extern MPU9250_WE myMPU9250;

void init_IMU();
void getValuesIMU(xyzFloat *accelOut, xyzFloat *gyroOut, xyzFloat *magOut, float *tempOut);

#endif
