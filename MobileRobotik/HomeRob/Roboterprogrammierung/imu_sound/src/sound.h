#ifndef Sound_H
#define Sound_H

#include <std_msgs/msg/string.h>

void initSound(std_msgs__msg__String &string_msg);
void SuperMario();
void IndianaJones();
void playCustomMelody(const char *payload);

#endif
