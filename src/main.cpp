#include <Arduino.h>
#include <SX126x-RAK4630.h>

#include "codec2.h"
#include "hts1a.h"

#define CODEC2_MODE CODEC2_MODE_2400
#define FRAMES_PER_PACKET 4

CODEC2* codec2 = nullptr;
int nsam = 0;
int nbyte = 0;
int sample_pos = 0;

static SemaphoreHandle_t c2_ready;
static SemaphoreHandle_t lora_ready;
static SemaphoreHandle_t enc_start;
static SemaphoreHandle_t enc_done;
static SemaphoreHandle_t tx_sem;  // signalled by OnTxDone / OnTxTimeout

short* pcm_buf = nullptr;
uint8_t* enc_buf = nullptr;

// Double buffer — one transmitting while the other is being filled
static uint8_t pkt_buf[2][FRAMES_PER_PACKET * 6];
static int pkt_len = 0;

// ── LoRa params ───────────────────────────────────────────────────────────
#define RF_FREQUENCY 868000000
#define TX_OUTPUT_POWER 22
#define LORA_BANDWIDTH 0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 1
#define LORA_PREAMBLE_LENGTH 8
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define TX_TIMEOUT_VALUE 3000

static RadioEvents_t RadioEvents;

void OnTxDone(void) { xSemaphoreGiveFromISR(tx_sem, NULL); }
void OnTxTimeout(void) { xSemaphoreGiveFromISR(tx_sem, NULL); }

// ── Codec2 tasks ──────────────────────────────────────────────────────────
void codec2_init_task(void* pv) {
    codec2 = codec2_create(CODEC2_MODE);
    if (codec2) {
        nsam = codec2_samples_per_frame(codec2);
        nbyte = (codec2_bits_per_frame(codec2) + 7) / 8;
        pcm_buf = (short*)malloc(nsam * sizeof(short));
        enc_buf = (uint8_t*)malloc(nbyte);
        codec2_set_natural_or_gray(codec2, 0);
        pkt_len = FRAMES_PER_PACKET * nbyte;
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

void encode_next_frame(void) {
    for (int i = 0; i < nsam; i++) {
        pcm_buf[i] = hts1a_samples[sample_pos++];
        if (sample_pos >= hts1a_num_samples) sample_pos = 0;
    }
    xSemaphoreGive(enc_start);
    xSemaphoreTake(enc_done, portMAX_DELAY);
}

void fill_packet(uint8_t* pkt) {
    for (int f = 0; f < FRAMES_PER_PACKET; f++) {
        encode_next_frame();
        memcpy(pkt + f * nbyte, enc_buf, nbyte);
    }
}

// ── LoRa init task ────────────────────────────────────────────────────────
void lora_init_task(void* pv) {
    lora_rak4630_init();
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(
        MODEM_LORA, TX_OUTPUT_POWER, 0,
        LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE,
        LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
        true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);
    xSemaphoreGive(lora_ready);
    vTaskDelete(NULL);
}

void tx_task(void* pv) {
    // Pre-fill buffer 0 before entering loop
    fill_packet(pkt_buf[0]);

    int tx_idx = 0;
    int enc_idx = 1;

    while (true) {
        // Start transmitting current packet
        uint32_t t0 = millis();
        Radio.Send(pkt_buf[tx_idx], pkt_len);

        // Immediately start encoding next packet in parallel with TX
        uint32_t t1 = millis();
        fill_packet(pkt_buf[enc_idx]);
        uint32_t enc_ms = millis() - t1;

        // Now wait for TX to finish (may already be done)
        xSemaphoreTake(tx_sem, portMAX_DELAY);
        uint32_t total_ms = millis() - t0;
        uint32_t tx_ms = total_ms - enc_ms;  // approximate

        Serial.printf("tx~=%lums enc=%lums cycle=%lums\n",
                      tx_ms, enc_ms, total_ms);

        tx_idx ^= 1;
        enc_idx ^= 1;
    }
}

// ── Setup ─────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    uint32_t t = millis();
    while (!Serial && (millis() - t) < 5000);
    delay(500);
    Serial.println("Codec2 LoRa TX");

    c2_ready = xSemaphoreCreateBinary();
    xTaskCreate(codec2_init_task, "c2init", 4096, NULL, 1, NULL);
    xSemaphoreTake(c2_ready, portMAX_DELAY);
    vSemaphoreDelete(c2_ready);

    if (!codec2 || !pcm_buf || !enc_buf) {
        Serial.println("Codec2 init failed");
        return;
    }
    Serial.printf("Codec2 ready: nsam=%d nbyte=%d\n", nsam, nbyte);

    enc_start = xSemaphoreCreateBinary();
    enc_done = xSemaphoreCreateBinary();
    xTaskCreate(codec2_encode_task, "c2enc", 4096, NULL, 1, NULL);

    tx_sem = xSemaphoreCreateBinary();

    lora_ready = xSemaphoreCreateBinary();
    xTaskCreate(lora_init_task, "lorainit", 4096, NULL, 1, NULL);
    xSemaphoreTake(lora_ready, portMAX_DELAY);
    vSemaphoreDelete(lora_ready);

    Serial.printf("LoRa ready: %.3f MHz SF%d BW125 %d frames/pkt\n",
                  RF_FREQUENCY / 1e6, LORA_SPREADING_FACTOR, FRAMES_PER_PACKET);

    xTaskCreate(tx_task, "tx", 4096, NULL, 2, NULL);
}

// ── Loop ──────────────────────────────────────────────────────────────────
void loop() {
    vTaskSuspend(NULL);
}