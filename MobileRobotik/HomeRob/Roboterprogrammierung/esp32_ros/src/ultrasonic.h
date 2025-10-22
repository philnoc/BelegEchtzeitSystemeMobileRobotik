#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>
#include <sensor_msgs/msg/range.h>

void initUltrasonic(sensor_msgs__msg__Range &range_msg);
long readDistanceCm();

#endif
