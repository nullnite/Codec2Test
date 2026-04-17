#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <SX126x-RAK4630.h>

#include "codec2.h"
#include "hts1a.h"

// ── Codec2 ────────────────────────────────────────────────────────────────
#define CODEC2_MODE CODEC2_MODE_2400
#define FRAMES_PER_PACKET 4  // 4 × 6 bytes = 24 bytes payload, 80ms audio per packet

CODEC2* codec2 = nullptr;
int nsam = 0;
int nbyte = 0;  // bytes per encoded frame (6 for 2400)
int sample_pos = 0;

static SemaphoreHandle_t c2_ready;
static SemaphoreHandle_t enc_start;
static SemaphoreHandle_t enc_done;

short* pcm_buf = nullptr;    // one frame of input PCM
uint8_t* enc_buf = nullptr;  // one frame of encoded output

// ── LoRa ──────────────────────────────────────────────────────────────────
#define RF_FREQUENCY 868000000
#define TX_OUTPUT_POWER 22
#define LORA_BANDWIDTH 0         // 125 kHz
#define LORA_SPREADING_FACTOR 7  // SF7 — lowest latency, highest throughput
#define LORA_CODINGRATE 1        // 4/5
#define LORA_PREAMBLE_LENGTH 8
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define TX_TIMEOUT_VALUE 3000

static RadioEvents_t RadioEvents;
static volatile bool tx_done = false;
static volatile bool tx_timeout = false;

void OnTxDone(void) {
    tx_done = true;
}

void OnTxTimeout(void) {
    tx_timeout = true;
}

// ── Codec2 tasks ──────────────────────────────────────────────────────────
void codec2_init_task(void* pv) {
    codec2 = codec2_create(CODEC2_MODE);
    if (codec2) {
        nsam = codec2_samples_per_frame(codec2);
        int nbit = codec2_bits_per_frame(codec2);
        nbyte = (nbit + 7) / 8;
        pcm_buf = (short*)malloc(nsam * sizeof(short));
        enc_buf = (uint8_t*)malloc(nbyte);
        codec2_set_natural_or_gray(codec2, 0);
    }
    xSemaphoreGive(c2_ready);
    vTaskDelete(NULL);
}

void codec2_encode_task(void* pv) {
    while (true) {
        xSemaphoreTake(enc_start, portMAX_DELAY);
        codec2_encode(codec2, enc_buf, pcm_buf);
        xSemaphoreGive(enc_done);
    }
}

// Encode one frame — fills pcm_buf from test audio, returns pointer to enc_buf
void encode_next_frame() {
    for (int i = 0; i < nsam; i++) {
        pcm_buf[i] = hts1a_samples[sample_pos++];
        if (sample_pos >= hts1a_num_samples) sample_pos = 0;
    }
    xSemaphoreGive(enc_start);
    xSemaphoreTake(enc_done, portMAX_DELAY);
}

// ── Setup ─────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    Serial.println("Codec2 LoRa TX");

    // Init codec2
    c2_ready = xSemaphoreCreateBinary();
    xTaskCreate(codec2_init_task, "c2init", 4096, NULL, 1, NULL);
    xSemaphoreTake(c2_ready, portMAX_DELAY);
    vSemaphoreDelete(c2_ready);

    if (!codec2 || !pcm_buf || !enc_buf) {
        Serial.println("Codec2 init failed");
        return;
    }
    Serial.printf("Codec2 ready: nsam=%d nbyte=%d\n", nsam, nbyte);

    // Spawn encode task
    enc_start = xSemaphoreCreateBinary();
    enc_done = xSemaphoreCreateBinary();
    xTaskCreate(codec2_encode_task, "c2enc", 4096, NULL, 1, NULL);

    // Init LoRa
    lora_rak4630_init();

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(
        MODEM_LORA,
        TX_OUTPUT_POWER,
        0,  // FSK deviation — unused for LoRa
        LORA_BANDWIDTH,
        LORA_SPREADING_FACTOR,
        LORA_CODINGRATE,
        LORA_PREAMBLE_LENGTH,
        LORA_FIX_LENGTH_PAYLOAD_ON,
        true,  // CRC on
        0, 0,  // freq hopping off
        LORA_IQ_INVERSION_ON,
        TX_TIMEOUT_VALUE);

    Serial.println("LoRa ready");
    Serial.printf("Freq: %.3f MHz  SF%d  BW125  %d frames/packet (%d bytes)\n",
                  RF_FREQUENCY / 1e6, LORA_SPREADING_FACTOR,
                  FRAMES_PER_PACKET, FRAMES_PER_PACKET * nbyte);
}

// ── Loop ──────────────────────────────────────────────────────────────────
void loop() {
    if (!codec2) return;

    // Build packet: encode FRAMES_PER_PACKET frames and concatenate
    uint8_t packet[FRAMES_PER_PACKET * 6];  // 6 bytes max per frame at 2400bps

    for (int f = 0; f < FRAMES_PER_PACKET; f++) {
        encode_next_frame();
        memcpy(packet + f * nbyte, enc_buf, nbyte);
    }

    int packet_len = FRAMES_PER_PACKET * nbyte;

    // Transmit
    tx_done = false;
    tx_timeout = false;
    Radio.Send(packet, packet_len);

    // Wait for TX done or timeout
    uint32_t t = millis();
    while (!tx_done && !tx_timeout) {
        delay(1);
    }

    uint32_t airtime = millis() - t;
    if (tx_done) {
        Serial.printf("TX OK  %d bytes  %lu ms airtime\n", packet_len, airtime);
    } else {
        Serial.println("TX TIMEOUT");
    }

    // Pace to audio rate: FRAMES_PER_PACKET frames × 20ms = 80ms per packet
    // Encoding takes ~90ms (4 × 22.55ms), so no extra delay needed —
    // the encode time itself paces the transmission
}