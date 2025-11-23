/* WS2812 RGB LED Blink Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#define MIC_ANALOG

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "sdkconfig.h"
#include <Arduino.h>
#include <FastLED.h>
#include <soc/adc_channel.h>
#include <driver/i2s.h>
#include <ggwave/ggwave.h>

/* WS2812 LED Configuration */
#define LED_PIN 16          // GPIO16 connected to WS2812 DIN
#define NUM_LEDS 1          // Number of WS2812 LEDs
#define LED_TYPE WS2812
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];

#ifndef LED_BUILTIN
#define LED_BUILTIN 4
#endif

// Pin configuration
const int kPinLED0 = 2;

// Global GGwave instance
GGWave ggwave;

// Audio capture configuration
using TSample      = int16_t;
#if defined(MIC_ANALOG)
using TSampleInput = int16_t;
#elif defined(MIC_I2S) || defined(MIC_I2S_SPH0645)
using TSampleInput = int32_t;
#endif

const size_t kSampleSize_bytes = sizeof(TSample);

// High sample rate - better quality, but more CPU/Memory usage
const int sampleRate = 24000;
const int samplesPerFrame = 512;

// Low sample rate
// Only MT protocols will work in this mode
//const int sampleRate = 12000;
//const int samplesPerFrame = 256;

TSample sampleBuffer[samplesPerFrame];

// helper buffer for data input in different formats:
#if defined(MIC_ANALOG)
TSampleInput * sampleBufferRaw = sampleBuffer;
#elif defined(MIC_I2S) || defined(MIC_I2S_SPH0645)
TSampleInput sampleBufferRaw[samplesPerFrame];
#endif

const i2s_port_t i2s_port = I2S_NUM_0;

#if defined(MIC_ANALOG)
// ADC configuration
const adc_unit_t     adc_unit    = ADC_UNIT_1;
const adc1_channel_t adc_channel = ADC1_GPIO35_CHANNEL;

// i2s config for using the internal ADC
const i2s_config_t i2s_config = {
    .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate          = sampleRate,
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format       = I2S_CHANNEL_FMT_ONLY_RIGHT,
    .communication_format = I2S_COMM_FORMAT_I2S_LSB,
    .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count        = 4,
    .dma_buf_len          = samplesPerFrame,
    .use_apll             = false,
    .tx_desc_auto_clear   = false,
    .fixed_mclk           = 0
};
#endif

void blink_task(void *pvParameter)
{
    /* Initialize FastLED */
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    FastLED.setBrightness(50);  // Set brightness (0-255)

    int colorIndex = 0;

    while(1) {
        switch(colorIndex % 7) {
            case 0:
                leds[0] = CRGB::Red;
                break;
            case 1:
                leds[0] = CRGB::Green;
                break;
            case 2:
                leds[0] = CRGB::Blue;
                break;
            case 3:
                leds[0] = CRGB::Yellow;
                break;
            case 4:
                leds[0] = CRGB::Purple;
                break;
            case 5:
                leds[0] = CRGB::Cyan;
                break;
            case 6:
                leds[0] = CRGB::White;
                break;
        }
        FastLED.show();
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        leds[0] = CRGB::Black;  // Turn off
        FastLED.show();
        vTaskDelay(500 / portTICK_PERIOD_MS);

        colorIndex++;
    }
}

#if !CONFIG_AUTOSTART_ARDUINO
void arduinoTask(void *pvParameter) {
    pinMode(LED_BUILTIN, OUTPUT);
    while(1) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(1000);
    }
}

extern "C" void app_main()
{
    // initialize arduino library before we start the tasks
    initArduino();

    xTaskCreate(&blink_task, "blink_task", 4096, NULL, 5, NULL);
    xTaskCreate(&arduinoTask, "arduino_task", 2048, NULL, 5, NULL);
}
#else
void setup() {
    Serial.begin(115200);
    xTaskCreate(&blink_task, "blink_task", 4096, NULL, 5, NULL);
    pinMode(LED_BUILTIN, OUTPUT);

    ggwave.setLogFile(nullptr);
    {

        auto p = GGWave::getDefaultParameters();

        // Adjust the "ggwave" parameters to your needs.
        // Make sure that the "payloadLength" parameter matches the one used on the transmitting side.
#ifdef LONG_RANGE
        // The "FAST" protocols require 2x more memory, so we reduce the payload length to compensate:
        p.payloadLength   = 8;
#else
        p.payloadLength   = 16;
#endif
        Serial.print(F("Using payload length: "));
        Serial.println(p.payloadLength);

        p.sampleRateInp   = sampleRate;
        p.sampleRateOut   = sampleRate;
        p.sampleRate      = sampleRate;
        p.samplesPerFrame = samplesPerFrame;
        p.sampleFormatInp = GGWAVE_SAMPLE_FORMAT_I16;
        p.sampleFormatOut = GGWAVE_SAMPLE_FORMAT_U8;
        p.operatingMode   = GGWAVE_OPERATING_MODE_RX | GGWAVE_OPERATING_MODE_TX | GGWAVE_OPERATING_MODE_USE_DSS | GGWAVE_OPERATING_MODE_TX_ONLY_TONES;

        // Protocols to use for TX
        // Remove the ones that you don't need to reduce memory usage
        GGWave::Protocols::tx().disableAll();
        //GGWave::Protocols::tx().toggle(GGWAVE_PROTOCOL_MT_NORMAL,  true);
        //GGWave::Protocols::tx().toggle(GGWAVE_PROTOCOL_MT_FAST,    true);
        GGWave::Protocols::tx().toggle(GGWAVE_PROTOCOL_MT_FASTEST, true);

        // Protocols to use for RX
        // Remove the ones that you don't need to reduce memory usage
        GGWave::Protocols::rx().disableAll();
        //GGWave::Protocols::rx().toggle(GGWAVE_PROTOCOL_DT_NORMAL,  true);
#ifdef LONG_RANGE
        GGWave::Protocols::rx().toggle(GGWAVE_PROTOCOL_DT_FAST,    true);
#endif
        GGWave::Protocols::rx().toggle(GGWAVE_PROTOCOL_DT_FASTEST, true);
        //GGWave::Protocols::rx().toggle(GGWAVE_PROTOCOL_MT_NORMAL,  true);
#ifdef LONG_RANGE
        GGWave::Protocols::rx().toggle(GGWAVE_PROTOCOL_MT_FAST,    true);
#endif
        GGWave::Protocols::rx().toggle(GGWAVE_PROTOCOL_MT_FASTEST, true);

        // Print the memory required for the "ggwave" instance:
        ggwave.prepare(p, false);

        Serial.print(F("Required memory by the ggwave instance: "));
        Serial.print(ggwave.heapSize());
        Serial.println(F(" bytes"));

        // Initialize the "ggwave" instance:
        ggwave.prepare(p, true);
        Serial.print(F("Instance initialized successfully! Memory used: "));
    }

    // Start capturing audio
    {
        Serial.println(F("Initializing I2S interface"));

        // Install and start i2s driver
        i2s_driver_install(i2s_port, &i2s_config, 0, NULL);

#if defined(MIC_ANALOG)
        Serial.println(F("Using analog input - initializing ADC"));

        // Init ADC pad
        i2s_set_adc_mode(adc_unit, adc_channel);

        // Enable the adc
        i2s_adc_enable(i2s_port);

        Serial.println(F("I2S ADC started"));
#endif

#if defined(MIC_I2S) || defined(MIC_I2S_SPH0645)
        Serial.println(F("Using I2S input"));

#if defined(MIC_I2S_SPH0645)
        Serial.println(F("Applying fix for SPH0645"));

        // https://github.com/atomic14/esp32_audio/blob/d2ac3490c0836cb46a69c83b0570873de18f695e/i2s_sampling/src/I2SMEMSSampler.cpp#L17-L22
        REG_SET_BIT(I2S_TIMING_REG(i2s_port), BIT(9));
        REG_SET_BIT(I2S_CONF_REG(i2s_port), I2S_RX_MSB_SHIFT);
#endif

        i2s_set_pin(i2s_port, &pin_config);
#endif
    }
}


int niter = 0;
int tLastReceive = -10000;

GGWave::TxRxData result;

void loop() {
    // Read from i2s
    {
        size_t bytes_read = 0;
        i2s_read(i2s_port, sampleBufferRaw, sizeof(TSampleInput)*samplesPerFrame, &bytes_read, portMAX_DELAY);

        int nSamples = bytes_read/sizeof(TSampleInput);
        if (nSamples != samplesPerFrame) {
            Serial.println("Failed to read samples");
            return;
        }

#if defined(MIC_ANALOG)
        // the ADC samples are 12-bit so we need to do some massaging to make them 16-bit
        for (int i = 0; i < nSamples; i += 2) {
            auto & s0 = sampleBuffer[i];
            auto & s1 = sampleBuffer[i + 1];

            s0 = s0 & 0x0fff;
            s1 = s1 & 0x0fff;

            s0 = s0 ^ s1;
            s1 = s0 ^ s1;
            s0 = s0 ^ s1;
        }
#endif

#if defined(MIC_I2S) || defined(MIC_I2S_SPH0645)
        for (int i = 0; i < nSamples; ++i) {
            sampleBuffer[i] = (sampleBufferRaw[i] & 0xFFFFFFF0) >> 11;
        }
#endif
    }

    // Use this with the serial plotter to observe real-time audio signal
    //for (int i = 0; i < nSamples; i++) {
    //    Serial.println(sampleBuffer[i]);
    //}

    // Try to decode any "ggwave" data:
    auto tStart = millis();

    if (ggwave.decode(sampleBuffer, samplesPerFrame*kSampleSize_bytes) == false) {
        Serial.println("Failed to decode");
    }

    auto tEnd = millis();

    if (++niter % 10 == 0) {
        // print the time it took the last decode() call to complete
        // should be smaller than samplesPerFrame/sampleRate seconds
        // for example: samplesPerFrame = 128, sampleRate = 6000 => not more than 20 ms
        Serial.println(tEnd - tStart);
        if (tEnd - tStart > 1000*(float(samplesPerFrame)/sampleRate)) {
            Serial.println(F("Warning: decode() took too long to execute!"));
        }
    }

    // Check if we have successfully decoded any data:
    int nr = ggwave.rxTakeData(result);
    if (nr > 0) {
        Serial.println(tEnd - tStart);
        Serial.print(F("Received data with length "));
        Serial.print(nr); // should be equal to p.payloadLength
        Serial.println(F(" bytes:"));

        Serial.println((char *) result.data());

        tLastReceive = tEnd;
    }
}
#endif
