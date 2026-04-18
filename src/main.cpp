#include <Arduino.h>
#include <SX126x-RAK4630.h>

#include "codec2.h"
#include "hts1a.h"

// ── Device role ───────────────────────────────────────────────────────────
#define ROLE_RX  // Change to ROLE_RX for the receiver

// ── Codec2 ────────────────────────────────────────────────────────────────
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

short* pcm_buf = nullptr;
uint8_t* enc_buf = nullptr;

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
#define RX_TIMEOUT_VALUE 0  // 0 = continuous receive

static RadioEvents_t RadioEvents;

// ── TX-specific ───────────────────────────────────────────────────────────
#ifdef ROLE_TX

static SemaphoreHandle_t tx_sem;
static uint8_t pkt_buf[2][FRAMES_PER_PACKET * 6];
static int pkt_len = 0;

void OnTxDone(void) { xSemaphoreGiveFromISR(tx_sem, NULL); }
void OnTxTimeout(void) { xSemaphoreGiveFromISR(tx_sem, NULL); }

#endif

// ── RX-specific ───────────────────────────────────────────────────────────
#ifdef ROLE_RX

// Jitter buffer
#define JITTER_FRAMES 8
#define PREBUFFER_FRAMES 3

static short jitter_buf[JITTER_FRAMES][160];
static short last_frame[160];
static volatile int jitter_write = 0;
static volatile int jitter_read = 0;
static volatile int jitter_count = 0;
static volatile bool playing = false;
static SoftwareTimer audio_timer;

static SemaphoreHandle_t rx_sem;
static uint8_t rx_packet[FRAMES_PER_PACKET * 6];
static int rx_packet_len = 0;
static int16_t rx_rssi = 0;
static int8_t rx_snr = 0;

// Decode task — large stack needed same as encode
static SemaphoreHandle_t dec_start;
static SemaphoreHandle_t dec_done;
static uint8_t* dec_frame_ptr = nullptr;  // points into rx_packet

void OnRxDone(uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr) {
    if (size <= sizeof(rx_packet)) {
        memcpy(rx_packet, payload, size);
        rx_packet_len = size;
        rx_rssi = rssi;
        rx_snr = snr;
    }
    // Re-arm immediately in the callback before giving semaphore
    Radio.Rx(RX_TIMEOUT_VALUE);
    xSemaphoreGiveFromISR(rx_sem, NULL);
}

void OnRxTimeout(void) {
    Radio.Rx(RX_TIMEOUT_VALUE);
    xSemaphoreGiveFromISR(rx_sem, NULL);
}

void OnRxError(void) {
    Radio.Rx(RX_TIMEOUT_VALUE);
    xSemaphoreGiveFromISR(rx_sem, NULL);
}

#endif

// ── Codec2 init task (shared) ─────────────────────────────────────────────
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

// ── Encode task (TX only) ─────────────────────────────────────────────────
#ifdef ROLE_TX

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

#endif

// ── Decode task (RX only) ─────────────────────────────────────────────────
#ifdef ROLE_RX

void codec2_decode_task(void* pv) {
    while (true) {
        xSemaphoreTake(dec_start, portMAX_DELAY);
        codec2_decode(codec2, pcm_buf, dec_frame_ptr);
        xSemaphoreGive(dec_done);
    }
}

// Decode one packet into jitter buffer
// After decoding each frame, also send raw PCM to serial:
void process_rx_packet(uint8_t* data, int len) {
    int frames = len / nbyte;
    for (int f = 0; f < frames; f++) {
        dec_frame_ptr = data + f * nbyte;
        xSemaphoreGive(dec_start);
        xSemaphoreTake(dec_done, portMAX_DELAY);

        // Feed jitter buffer
        if (jitter_count < JITTER_FRAMES) {
            memcpy(jitter_buf[jitter_write], pcm_buf, nsam * sizeof(short));
            memcpy(last_frame, pcm_buf, nsam * sizeof(short));
            jitter_write = (jitter_write + 1) % JITTER_FRAMES;
            jitter_count++;
        }

        // Stream raw PCM to serial for PC playback verification
        Serial.write(0xFF);  // frame start marker
        Serial.write(0xFE);  // second marker byte (0xFE unlikely in text)
        Serial.write((uint8_t*)pcm_buf, nsam * sizeof(short));
    }
    if (!playing && jitter_count >= PREBUFFER_FRAMES)
        playing = true;
}

// Pull next frame from jitter buffer — call this from audio output ISR/timer
// Returns pointer to nsam samples of 16-bit PCM at 8kHz
const short* get_next_audio_frame(void) {
    if (!playing) return last_frame;

    if (jitter_count > 0) {
        const short* frame = jitter_buf[jitter_read];
        jitter_read = (jitter_read + 1) % JITTER_FRAMES;
        jitter_count--;
        return frame;
    }
    // Underrun — repeat last frame (packet loss concealment)
    Serial.println("jitter underrun — repeating frame");
    return last_frame;
}

void audio_timer_cb(TimerHandle_t xTimer) {
    if (!playing) return;
    const short* frame = get_next_audio_frame();
    // TODO: send frame to I2S DAC here
    // For now just track stats
    static int frame_count = 0;
    static int underruns = 0;
    frame_count++;
    // underrun detection already prints in get_next_audio_frame
}

#endif

// ── LoRa init task (shared) ───────────────────────────────────────────────
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

// ── TX task ───────────────────────────────────────────────────────────────
#ifdef ROLE_TX

void tx_task(void* pv) {
    fill_packet(pkt_buf[0]);

    int tx_idx = 0;
    int enc_idx = 1;

    while (true) {
        uint32_t t0 = millis();
        Radio.Send(pkt_buf[tx_idx], pkt_len);

        uint32_t t1 = millis();
        fill_packet(pkt_buf[enc_idx]);
        uint32_t enc_ms = millis() - t1;

        xSemaphoreTake(tx_sem, portMAX_DELAY);
        uint32_t cycle_ms = millis() - t0;

        Serial.printf("enc=%lums cycle=%lums jitter=%d\n",
                      enc_ms, cycle_ms, 0);

        tx_idx ^= 1;
        enc_idx ^= 1;
    }
}

#endif

// ── RX task ───────────────────────────────────────────────────────────────
#ifdef ROLE_RX

void rx_task(void* pv) {
    Radio.Rx(RX_TIMEOUT_VALUE);  // initial arm only
    Serial.println("Listening...");

    while (true) {
        xSemaphoreTake(rx_sem, portMAX_DELAY);

        if (rx_packet_len > 0) {
            Serial.printf("RX %d bytes  RSSI=%d  SNR=%d  jitter=%d\n",
                          rx_packet_len, rx_rssi, rx_snr, jitter_count);

            process_rx_packet(rx_packet, rx_packet_len);
            rx_packet_len = 0;

            for (int f = 0; f < FRAMES_PER_PACKET; f++) {
                int slot = (jitter_write - FRAMES_PER_PACKET + f + JITTER_FRAMES) % JITTER_FRAMES;
                Serial.printf("  frame %d: %d %d %d %d ...\n", f,
                              jitter_buf[slot][0], jitter_buf[slot][1],
                              jitter_buf[slot][2], jitter_buf[slot][3]);
            }
        }
        // No Radio.Rx() here — already re-armed in callback
    }
}

#endif

// ── Setup ─────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    uint32_t t = millis();
    while (!Serial && (millis() - t) < 5000);
    delay(500);

#ifdef ROLE_TX
    Serial.println("=== Codec2 LoRa TX ===");
#else
    Serial.println("=== Codec2 LoRa RX ===");
#endif

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

    // Spawn encode or decode task
#ifdef ROLE_TX
    enc_start = xSemaphoreCreateBinary();
    enc_done = xSemaphoreCreateBinary();
    xTaskCreate(codec2_encode_task, "c2enc", 4096, NULL, 1, NULL);
#else
    dec_start = xSemaphoreCreateBinary();
    dec_done = xSemaphoreCreateBinary();
    xTaskCreate(codec2_decode_task, "c2dec", 4096, NULL, 1, NULL);
#endif

    // Init LoRa
    lora_ready = xSemaphoreCreateBinary();
    xTaskCreate(lora_init_task, "lorainit", 4096, NULL, 1, NULL);
    xSemaphoreTake(lora_ready, portMAX_DELAY);
    vSemaphoreDelete(lora_ready);

    Serial.printf("LoRa ready: %.3f MHz SF%d BW125 %d frames/pkt\n",
                  RF_FREQUENCY / 1e6, LORA_SPREADING_FACTOR, FRAMES_PER_PACKET);

    // Spawn main task
#ifdef ROLE_TX
    tx_sem = xSemaphoreCreateBinary();
    xTaskCreate(tx_task, "tx", 4096, NULL, 3, NULL);
#else
    rx_sem = xSemaphoreCreateBinary();
    xTaskCreate(rx_task, "rx", 4096, NULL, 3, NULL);
    audio_timer.begin(20, audio_timer_cb);
    audio_timer.start();
#endif
}

void loop() {
    vTaskSuspend(NULL);
}