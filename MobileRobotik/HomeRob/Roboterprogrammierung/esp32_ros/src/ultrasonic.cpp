#include "ultrasonic.h"

#define TRIGGER_PIN 26
#define ECHO_PIN 25


void initUltrasonic(sensor_msgs__msg__Range &range_msg) {
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    range_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    range_msg.field_of_view = 0.5;
    range_msg.min_range = 0.02;
    range_msg.max_range = 4.0;
    range_msg.header.frame_id.data = (char *)"ultrasonic_sensor";
    range_msg.header.frame_id.size = strlen("ultrasonic_sensor");
    range_msg.header.frame_id.capacity = range_msg.header.frame_id.size + 1;
}

long readDistanceCm() {
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Timeout 30ms

    if (duration == 0) return -1;

    return duration / 29 / 2;
}

