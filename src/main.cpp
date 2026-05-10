#include <Arduino.h>
#include <SX126x-RAK4630.h>

#include "codec2.h"
#include "hts1a.h"

#define ROLE_TX  // Change to ROLE_TX for the transmitter

#define CODEC2_MODE CODEC2_MODE_2400
#define FRAMES_PER_PACKET 4

CODEC2* codec2 = nullptr;
int nsam = 0;
int nbyte = 0;

static SemaphoreHandle_t c2_ready;
static SemaphoreHandle_t lora_ready;

short* pcm_buf = nullptr;
uint8_t* enc_buf = nullptr;

#define RF_FREQUENCY 868000000
#define TX_OUTPUT_POWER 22
#define LORA_BANDWIDTH 0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 1
#define LORA_PREAMBLE_LENGTH 8
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define LORA_SYMBOL_TIMEOUT 0
#define TX_TIMEOUT_VALUE 3000
#define RX_TIMEOUT_VALUE 0

static RadioEvents_t RadioEvents;

// ── TX ────────────────────────────────────────────────────────────────────
#ifdef ROLE_TX

static SemaphoreHandle_t enc_start;
static SemaphoreHandle_t enc_done;
static SemaphoreHandle_t tx_sem;
static uint8_t pkt_buf[2][FRAMES_PER_PACKET * 6];
static int pkt_len = 0;
static int sample_pos = 0;

void OnTxDone(void) { xSemaphoreGiveFromISR(tx_sem, NULL); }
void OnTxTimeout(void) { xSemaphoreGiveFromISR(tx_sem, NULL); }

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

void tx_task(void* pv) {
    const TickType_t interval = pdMS_TO_TICKS(FRAMES_PER_PACKET * 20);
    TickType_t last_wake = xTaskGetTickCount();
    fill_packet(pkt_buf[0]);
    int tx_idx = 0;
    int enc_idx = 1;
    bool first = true;
    while (true) {
        vTaskDelayUntil(&last_wake, interval);
        if (first) { Serial.println("TX start"); first = false; }
        Radio.Send(pkt_buf[tx_idx], pkt_len);
        fill_packet(pkt_buf[enc_idx]);
        xSemaphoreTake(tx_sem, portMAX_DELAY);
        tx_idx ^= 1;
        enc_idx ^= 1;
    }
}

#endif  // ROLE_TX

// ── RX ────────────────────────────────────────────────────────────────────
#ifdef ROLE_RX

#define JITTER_FRAMES 8
#define PREBUFFER_FRAMES 3

static short jitter_buf[JITTER_FRAMES][160];
static short last_frame[160];
static volatile int jitter_write = 0;
static volatile int jitter_read = 0;
static volatile int jitter_count = 0;
static volatile bool playing = false;

static SemaphoreHandle_t dec_start;
static SemaphoreHandle_t dec_done;
static QueueHandle_t pcm_out_q;
static QueueHandle_t raw_pkt_q;

typedef struct {
    uint8_t data[FRAMES_PER_PACKET * 6];
    int len;
    int16_t rssi;
} RawPacket;

// dec_frame_ptr is local to decode_task — no longer a global
static SemaphoreHandle_t dec_frame_sem;
static uint8_t* dec_frame_ptr = nullptr;

void OnRxDone(uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr) {
    RawPacket pkt;
    pkt.len = 0;
    pkt.rssi = rssi;
    if (size <= sizeof(pkt.data)) {
        memcpy(pkt.data, payload, size);
        pkt.len = (int)size;
    }
    Radio.Rx(RX_TIMEOUT_VALUE);
    xQueueSendFromISR(raw_pkt_q, &pkt, NULL);
}

void OnRxTimeout(void) { Radio.Rx(RX_TIMEOUT_VALUE); }
void OnRxError(void) { Radio.Rx(RX_TIMEOUT_VALUE); }

void codec2_decode_task(void* pv) {
    while (true) {
        xSemaphoreTake(dec_start, portMAX_DELAY);
        codec2_decode(codec2, pcm_buf, dec_frame_ptr);
        xSemaphoreGive(dec_done);
    }
}

void process_rx_packet(const uint8_t* data, int len) {
    int frames = len / nbyte;
    for (int f = 0; f < frames; f++) {
        // Point decode task at this frame — safe because decode_task
        // only runs when we give dec_start and we wait for dec_done
        dec_frame_ptr = (uint8_t*)(data + f * nbyte);
        xSemaphoreGive(dec_start);
        xSemaphoreTake(dec_done, portMAX_DELAY);

        // pcm_buf now holds decoded frame
        xQueueSend(pcm_out_q, pcm_buf, 0);

        if (jitter_count < JITTER_FRAMES) {
            memcpy(jitter_buf[jitter_write], pcm_buf, nsam * sizeof(short));
            memcpy(last_frame, pcm_buf, nsam * sizeof(short));
            jitter_write = (jitter_write + 1) % JITTER_FRAMES;
            jitter_count++;
        }
    }
    if (!playing && jitter_count >= PREBUFFER_FRAMES)
        playing = true;
}

void decode_task(void* pv) {
    RawPacket pkt;
    while (true) {
        xQueueReceive(raw_pkt_q, &pkt, portMAX_DELAY);
        if (pkt.len > 0)
            process_rx_packet(pkt.data, pkt.len);
    }
}

void serial_output_task(void* pv) {
    short frame[160];
    uint8_t header[6];
    uint8_t seq = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        xQueueReceive(pcm_out_q, frame, portMAX_DELAY);

        // Compute XOR checksum over raw PCM bytes
        uint8_t* raw = (uint8_t*)frame;
        uint8_t chk = 0;
        for (int i = 0; i < 160 * 2; i++) chk ^= raw[i];

        header[0] = 0xFF;
        header[1] = 0xFE;
        header[2] = seq++;                                 // sequence number — wraps 0-255
        uint16_t frame_bytes = 160 * 2;                    // 320, computed as 16-bit
        header[3] = (uint8_t)(frame_bytes & 0xFF);         // 0x40
        header[4] = (uint8_t)((frame_bytes >> 8) & 0xFF);  // 0x01
        header[5] = chk;                                   // XOR checksum

        Serial.write(header, 6);
        Serial.write(raw, 160 * 2);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
    }
}

const short* get_next_audio_frame(void) {
    if (!playing) return last_frame;
    if (jitter_count > 0) {
        const short* frame = jitter_buf[jitter_read];
        jitter_read = (jitter_read + 1) % JITTER_FRAMES;
        jitter_count--;
        return frame;
    }
    return last_frame;
}

#endif  // ROLE_RX

// ── Shared ────────────────────────────────────────────────────────────────
void codec2_init_task(void* pv) {
    codec2 = codec2_create(CODEC2_MODE);
    if (codec2) {
        nsam = codec2_samples_per_frame(codec2);
        nbyte = (codec2_bits_per_frame(codec2) + 7) / 8;
        pcm_buf = (short*)malloc(nsam * sizeof(short));
        enc_buf = (uint8_t*)malloc(nbyte);
        codec2_set_natural_or_gray(codec2, 0);
#ifdef ROLE_TX
        pkt_len = FRAMES_PER_PACKET * nbyte;
#endif
    }
    xSemaphoreGive(c2_ready);
    vTaskDelete(NULL);
}

void lora_init_task(void* pv) {
    lora_rak4630_init();
#ifdef ROLE_TX
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
#endif
#ifdef ROLE_RX
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;
#endif
    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
#ifdef ROLE_TX
    Radio.SetTxConfig(
        MODEM_LORA, TX_OUTPUT_POWER, 0,
        LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE,
        LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
        true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);
#endif
#ifdef ROLE_RX
    Radio.SetRxConfig(
        MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE,
        0, LORA_PREAMBLE_LENGTH, LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
        0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
#endif
    xSemaphoreGive(lora_ready);
    vTaskDelete(NULL);
}

void setup() {
    Serial.begin(921600);
    uint32_t t = millis();
    while (!Serial && (millis() - t) < 5000);
    delay(500);

#ifdef ROLE_TX
    Serial.println("=== Codec2 LoRa TX ===");
#else
    Serial.println("=== Codec2 LoRa RX ===");
#endif

    c2_ready = xSemaphoreCreateBinary();
    xTaskCreate(codec2_init_task, "c2init", 4096, NULL, 1, NULL);
    xSemaphoreTake(c2_ready, portMAX_DELAY);
    vSemaphoreDelete(c2_ready);

    if (!codec2 || !pcm_buf || !enc_buf) {
        Serial.println("Codec2 init failed");
        return;
    }

    lora_ready = xSemaphoreCreateBinary();
    xTaskCreate(lora_init_task, "lorainit", 4096, NULL, 1, NULL);
    xSemaphoreTake(lora_ready, portMAX_DELAY);
    vSemaphoreDelete(lora_ready);

#ifdef ROLE_TX
    enc_start = xSemaphoreCreateBinary();
    enc_done = xSemaphoreCreateBinary();
    xTaskCreate(codec2_encode_task, "c2enc", 4096, NULL, 1, NULL);
    tx_sem = xSemaphoreCreateBinary();
    xTaskCreate(tx_task, "tx", 4096, NULL, 3, NULL);
#endif

#ifdef ROLE_RX
    dec_start = xSemaphoreCreateBinary();
    dec_done = xSemaphoreCreateBinary();
    xTaskCreate(codec2_decode_task, "c2dec", 4096, NULL, 1, NULL);
    raw_pkt_q = xQueueCreate(300, sizeof(RawPacket));
    xTaskCreate(decode_task, "decode", 4096, NULL, 2, NULL);
    pcm_out_q = xQueueCreate(16, nsam * sizeof(short));
    xTaskCreate(serial_output_task, "serout", 4096, NULL, 1, NULL);
    Radio.Rx(RX_TIMEOUT_VALUE);
#endif
}

void loop() {
    vTaskSuspend(NULL);
}