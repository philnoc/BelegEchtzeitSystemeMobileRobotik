#include "sound.h"
#include <Arduino.h>

#define Buzzer 4
#define REST 0

#define NOTE_E7 2637 
#define NOTE_G7 3136 
#define NOTE_A7 3520 
#define NOTE_B7 3951 
#define NOTE_E6 1319 
#define NOTE_G6 1568 
#define NOTE_A6 1760 
#define NOTE_C7 2093 
#define REST 0 

// Super Mario Melodie 
const int Mariomelody[] = { NOTE_E7, NOTE_E7, REST, NOTE_E7, REST, NOTE_C7, NOTE_E7, REST, NOTE_G7, REST, REST, REST, NOTE_G6, REST, REST, REST }; 
const int MarionoteDurations[] = { 12, 12, 12, 12, 12, 12, 12, 12, 9, 12, 12, 12, 12, 12, 12, 12 }; 

// Indiana Jones Melodie 
const int Jonesnotes[] = { 330, 349, 392, 523, 294, 330, 349, 392, 440, 494, 698, 440, 494, 523, 587, 659, 330, 349, 392, 523, 587, 659, 698, 392, 392, 659, 587, 392, 659, 587, 392, 659, 587, 392, 698, 659, 587, 523 }; 
const int Jonesdurations[] = { 300, 100, 400, 800, 300, 100, 1200, 300, 100, 400, 800, 300, 100, 400, 400, 400, 300, 100, 400, 800, 300, 100, 1200, 300, 100, 400, 300, 100, 400, 300, 100, 400, 300, 100, 400, 300, 100, 1200 };
void initSound(std_msgs__msg__String &string_msg){
  // Speicher für Sound-Strings
  string_msg.data.data = (char *)malloc(100);  // Speicher reservieren
  string_msg.data.size = 0;                    // Anfangsgröße = 0
  string_msg.data.capacity = 100;              // Kapazität für bis zu 100 Zeichen
}
void playMelody(const int* notes, const int* durations, int length, float delayFactor = 1.30) {
  for (int i = 0; i < length; i++) {
    int note = notes[i];
    int duration = durations[i];

    int noteDuration = (duration > 0 && duration < 1000) ? 1000 / duration : duration;
    if (note != REST) {
      tone(Buzzer, note, noteDuration);
    }
    delay(noteDuration * delayFactor);
    noTone(Buzzer);
  }
}

void SuperMario() {
  int len = sizeof(Mariomelody) / sizeof(Mariomelody[0]);
  playMelody(Mariomelody, MarionoteDurations, len);
}

void IndianaJones() {
  int len = sizeof(Jonesnotes) / sizeof(Jonesnotes[0]);
  playMelody(Jonesnotes, Jonesdurations, len, 1.0);
}

// Hilfsfunktion: CSV-String zu Array
int parseCSV(String input, int *output, int maxCount) {
  int count = 0;
  int start = 0;
  while (true) {
    int idx = input.indexOf(',', start);
    if (idx == -1) {
      if (count < maxCount) output[count++] = input.substring(start).toInt();
      break;
    } else {
      if (count < maxCount) output[count++] = input.substring(start, idx).toInt();
      start = idx + 1;
    }
  }
  return count;
}

// Neue Funktion für Custom-Melodien
void playCustomMelody(const char *payload) {
  String s(payload);
  int sep = s.indexOf(';');
  if (sep == -1) {
    Serial.println("Ungültiges Format! Erwartet: notes:...,;durations:...");
    return;
  }

  String notes_str = s.substring(s.indexOf(':') + 1, sep);
  String dur_str   = s.substring(s.lastIndexOf(':') + 1);

  Serial.println("Notes-String: " + notes_str);
  Serial.println("Durations-String: " + dur_str);
  
  int notes[50];
  int durations[50];
  int nCount = parseCSV(notes_str, notes, 50);
  int dCount = parseCSV(dur_str, durations, 50);

  int length = min(nCount, dCount);
  Serial.printf("Custom Melody mit %d Noten\n", length);

  playMelody(notes, durations, length);
}
