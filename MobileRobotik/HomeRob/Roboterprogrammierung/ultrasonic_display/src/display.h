#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <std_msgs/msg/string.h>

void initDisplay(std_msgs__msg__String &string_msg);
void showMessage(const String &msg);

#endif
