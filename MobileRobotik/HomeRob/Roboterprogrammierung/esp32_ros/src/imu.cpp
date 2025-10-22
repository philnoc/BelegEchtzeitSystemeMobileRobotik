#include "imu.h"

MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);


void init_IMU() {
    // Initialisierung
    if(!myMPU9250.init()) {
        Serial.println("MPU9250 does not respond");
    } else {
        Serial.println("MPU9250 is connected");
    }

    if(!myMPU9250.initMagnetometer()) {
        Serial.println("Magnetometer does not respond");
    } else {
        Serial.println("Magnetometer is connected");
    }

    // Kalibrierung
    Serial.println("Position your MPU9250 flat and don't move it - calibrating...");
    delay(1000);
    myMPU9250.autoOffsets();
    Serial.println("Done!");

    // Gyro-Filter aktivieren
    myMPU9250.enableGyrDLPF();
    myMPU9250.setGyrDLPF(MPU9250_DLPF_6);

    // Sample Rate
    myMPU9250.setSampleRateDivider(5);

    // Gyro-Bereich
    myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);

    // Accel-Bereich
    myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);

    // Accel-Filter aktivieren
    myMPU9250.enableAccDLPF(true);
    myMPU9250.setAccDLPF(MPU9250_DLPF_6);

    // Magnetometer Mode
    myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);

    delay(200); // Zeit für Stabilisierung
}

void getValuesIMU (xyzFloat *accelOut, xyzFloat *gyroOut, xyzFloat *magOut, float *tempOut){
    *accelOut = myMPU9250.getGValues();
    *gyroOut = myMPU9250.getGyrValues();
    *magOut = myMPU9250.getMagValues();
    *tempOut = myMPU9250.getTemperature();
}
