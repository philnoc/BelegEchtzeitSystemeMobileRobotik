#include "display.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_SDA      21
#define OLED_SCL      22

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void initDisplay(std_msgs__msg__String &string_msg) {
  Wire.begin(OLED_SDA, OLED_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.println("Display bereit!");
  display.display();

  string_msg.data.data = (char *)malloc(100);  // Speicher reservieren
  string_msg.data.size = 0;                    // Anfangsgröße = 0
  string_msg.data.capacity = 100;              // Kapazität für bis zu 100 Zeichen
}

void showMessage(const String &msg) {
  
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.println(msg);
  display.display();
}
