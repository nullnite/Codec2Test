#include <Adafruit_TinyUSB.h>
#include <Arduino.h>

void setup() {
    Serial.begin(115200);
}

void loop() {
    Serial.println(millis());
}