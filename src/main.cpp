#include <Adafruit_TinyUSB.h>
#include <Arduino.h>

#include "codec2.h"

CODEC2* codec2 = nullptr;
volatile bool codec2_ready = false;

void codec2_task(void* pv) {
    codec2 = codec2_create(CODEC2_MODE_2400);
    codec2_ready = true;
    vTaskDelete(NULL);
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("Starting codec2 init task...");

    // Give this task 16KB of stack — plenty for nlp_create internals
    xTaskCreate(
        codec2_task,
        "codec2init",
        4096,  // stack in 32-bit words = 16KB
        NULL,
        1,
        NULL);
}

void loop() {
    if (codec2_ready && codec2) {
        Serial.println("codec2_create OK");
        codec2_ready = false;  // print once
    }
}