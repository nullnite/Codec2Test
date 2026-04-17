#include <Adafruit_TinyUSB.h>
#include <Arduino.h>

#include "codec2.h"
#include "hts1a.h"

#define CODEC2_MODE CODEC2_MODE_2400

CODEC2* codec2 = nullptr;
short* pcm_buf = nullptr;
int nsam = 0;
int nbyte = 0;

// Encoded output — one contiguous buffer for all frames
uint8_t* enc_out = nullptr;
int total_frames = 0;

static SemaphoreHandle_t c2_done;
static SemaphoreHandle_t enc_start;
static SemaphoreHandle_t enc_done;
static uint8_t* enc_frame_ptr = nullptr;  // points into enc_out for current frame

void codec2_init_task(void* pv) {
    codec2 = codec2_create(CODEC2_MODE);

    if (codec2) {
        nsam = codec2_samples_per_frame(codec2);
        int nbit = codec2_bits_per_frame(codec2);
        nbyte = (nbit + 7) / 8;
        total_frames = hts1a_num_samples / nsam;

        pcm_buf = (short*)malloc(nsam * sizeof(short));
        enc_out = (uint8_t*)malloc(total_frames * nbyte);
        codec2_set_natural_or_gray(codec2, 0);
    }

    xSemaphoreGive(c2_done);
    vTaskDelete(NULL);
}

void codec2_encode_task(void* pv) {
    while (true) {
        xSemaphoreTake(enc_start, portMAX_DELAY);
        codec2_encode(codec2, enc_frame_ptr, pcm_buf);
        xSemaphoreGive(enc_done);
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    c2_done = xSemaphoreCreateBinary();
    xTaskCreate(codec2_init_task, "c2init", 4096, NULL, 1, NULL);
    xSemaphoreTake(c2_done, portMAX_DELAY);
    vSemaphoreDelete(c2_done);

    if (!codec2 || !pcm_buf || !enc_out) {
        Serial.println("codec2 init failed");
        return;
    }

    enc_start = xSemaphoreCreateBinary();
    enc_done = xSemaphoreCreateBinary();
    xTaskCreate(codec2_encode_task, "c2enc", 4096, NULL, 1, NULL);

    int sample_pos = 0;
    Serial.print("Encoding ");
    Serial.print(total_frames);
    Serial.println(" frames...");
    Serial.flush();

    uint32_t t0 = millis();

    for (int f = 0; f < total_frames; f++) {
        for (int i = 0; i < nsam; i++)
            pcm_buf[i] = hts1a_samples[sample_pos++];

        enc_frame_ptr = enc_out + f * nbyte;
        xSemaphoreGive(enc_start);
        xSemaphoreTake(enc_done, portMAX_DELAY);
    }

    uint32_t elapsed = millis() - t0;
    Serial.print("Done in ");
    Serial.print(elapsed);
    Serial.print(" ms (");
    Serial.print((float)elapsed / total_frames, 2);
    Serial.println(" ms/frame avg)");

    // Dump encoded output as raw binary over Serial
    // Read on PC with: python -c "import serial; ...)" or just capture the port
    Serial.println("BEGIN_C2");
    Serial.write(enc_out, total_frames * nbyte);
    Serial.println("\nEND_C2");
}

void loop() {}
