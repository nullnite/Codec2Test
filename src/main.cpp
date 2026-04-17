/*#include <Adafruit_TinyUSB.h>
#include <Arduino.h>

#include "codec2.h"

const int mode = CODEC2_MODE_700B;
const int natural = 1;

CODEC2* codec2;
int nsam, nbit, nbyte, i, frames, bits_proc, bit_errors, error_mode;
int nstart_bit, nend_bit, bit_rate;
short* buf;
unsigned char* bits;
float* softdec_bits;

uint8_t encoded[32] = {0};
short mic_buf[320] = {0};

void setup() {
    Serial.begin(115200);

    // Wait for USB serial connection
    while (!Serial) {
        delay(10);
    }

    Serial.println("Test1");

    codec2 = codec2_create(mode);
    Serial.println("Testcreate");
    nsam = codec2_samples_per_frame(codec2);
    Serial.println("Testsmpl");
    nbit = codec2_bits_per_frame(codec2);
    Serial.println("Testbits");
    buf = (short*)malloc(nsam * sizeof(short));
    nbyte = (nbit + 7) / 8;
    bits = (unsigned char*)malloc(nbyte * sizeof(char));
    softdec_bits = (float*)malloc(nbit * sizeof(float));
    frames = bit_errors = bits_proc = 0;
    nstart_bit = 0;
    nend_bit = nbit - 1;

    Serial.println("Test2");

    codec2_set_natural_or_gray(codec2, !natural);

    Serial.println("Test3");
}

void loop() {
    // uint32_t time_start = millis();
    //  codec2_encode(codec2, encoded, mic_buf);
    // uint32_t time_end = millis();

    // Serial.println(time_end - time_start);

    //Serial.println("Test");

    //delay(100);
}*/

#include "Adafruit_TinyUSB.h"
#include "codec2.h"

bool storageReady = false;

const int mode = CODEC2_MODE_700B;
const int natural = 1;

CODEC2* codec2;
int nsam, nbit, nbyte, i, frames, bits_proc, bit_errors, error_mode;
int nstart_bit, nend_bit, bit_rate;
short* buf;
unsigned char* bits;
float* softdec_bits;

void recitation() { /*
                     #ifdef TESTING
                       Serial.println("Begin recitation");
                     #endif
                     uint8_t c2Buf[nbyte];
                     uint8_t maxlen = sizeof(c2Buf);
                     uint8_t offset=0;

                     int16_t audioBuf[nsam];

                     SD.begin(SD_CS);
                     File c2File = SD.open("TEST700B.C2");
                     if (c2File) {
                       #ifdef TESTING
                         Serial.println("Reciting Codec2 file");
                       #endif

                       // read from the file until there's nothing else in it:
                       while (c2File.available()) {
                         #ifdef TESTING
                           Serial.print("~");
                         #endif

                         offset=0;
                         for(int x=0; x<maxlen; x++) {
                           if(!c2File.available()) {
                             #ifdef TESTING
                               Serial.println("EOF");
                             #endif
                             break;
                           }

                           c2Buf[offset++] = c2File.read();
                         }

                         if(offset!=nbyte)
                         {
                           break;
                         }

                         codec2_decode(codec2, audioBuf, c2Buf);

                         for(int x=0; x<nsam; x++)
                         {
                           uint16_t sample=audioBuf[x];
                           char lo = sample & 0xFF;
                           char hi = sample >> 8;
                           Serial.print(lo);
                           Serial.print(hi);
                         }
                       }

                       // close the file:
                       c2File.close();
                       #ifdef TESTING
                         Serial.println("Recitation complete.");
                       #endif
                     } else {
                       // if the file didn't open, print an error:
                       Serial.println("Error opening Codec2 file");
                     }
                   */
}

void setup() {
    Serial.begin(9600);

    Serial.println("test");

    while (!Serial) {
        delay(10);
    }

    Serial.println("test2");

    // unsigned long startTime = millis();
    //  while (!Serial && (startTime + (10 * 1000) > millis())) {}

    codec2 = codec2_create(mode);
    Serial.println("test after codec");
    nsam = codec2_samples_per_frame(codec2);
    nbit = codec2_bits_per_frame(codec2);
    buf = (short*)malloc(nsam * sizeof(short));
    nbyte = (nbit + 7) / 8;
    bits = (unsigned char*)malloc(nbyte * sizeof(char));
    softdec_bits = (float*)malloc(nbit * sizeof(float));
    frames = bit_errors = bits_proc = 0;
    nstart_bit = 0;
    nend_bit = nbit - 1;

    codec2_set_natural_or_gray(codec2, !natural);

    if (storageReady) {
        recitation();
    }

    Serial.println("test after setup");

    codec2_destroy(codec2);

    Serial.println("test after destroy");
}

void loop() {
    Serial.println(millis());
}