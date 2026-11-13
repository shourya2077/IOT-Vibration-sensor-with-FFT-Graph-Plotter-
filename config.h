/*
 * config.h
 * Central place for all tunable constants. Change things here first
 * before touching any class code.
 */
#ifndef CONFIG_H_
#define CONFIG_H_

#include <Arduino.h>
#include "freertos/FreeRTOS.h"  // for BaseType_t / UBaseType_t below

// ---------------------------------------------------------------------
// I2C / sensor wiring (ESP32 DevKit V1, 30-pin)
// MPU6500: VCC->3V3, GND->GND, SCL->GPIO22, SDA->GPIO21, AD0->GND (addr 0x68)
// ---------------------------------------------------------------------
static const int PIN_SDA        = 21;
static const int PIN_SCL        = 22;
static const uint32_t I2C_CLOCK_HZ = 400000;   // 400kHz fast mode

// ---------------------------------------------------------------------
// Sampling / FFT configuration
// ---------------------------------------------------------------------
// MPU6500 accel DLPF only exposes 184/92/41/20/10/5 Hz bandwidths in this
// driver (no "wide open" option). 184Hz is the widest available, so real
// usable vibration bandwidth is roughly DC-180Hz even though we sample
// faster. This part is a MEMS motion sensor, not a dedicated vibration
// accelerometer -- fine for imbalance/looseness/misalignment type signatures,
// NOT for high-frequency bearing-defect envelope analysis (that needs a
// piezo accelerometer with a flat response into the kHz range).
static const uint16_t SAMPLE_RATE_HZ   = 1000;   // accel output rate (SRD=0 -> 1000/(1+0))
static const uint16_t FFT_SIZE         = 256;    // MUST be a power of 2
// Update rate = SAMPLE_RATE_HZ / FFT_SIZE ~= 3.9 Hz refresh with the defaults.
// Bigger FFT_SIZE -> finer frequency resolution but slower refresh.
// Frequency resolution = SAMPLE_RATE_HZ / FFT_SIZE (Hz/bin)

// ---------------------------------------------------------------------
// FreeRTOS task layout
// ---------------------------------------------------------------------
static const BaseType_t SAMPLING_TASK_CORE   = 1;   // isolated from WiFi stack
static const BaseType_t PROCESSING_TASK_CORE = 0;   // shares core with AsyncTCP
static const UBaseType_t SAMPLING_TASK_PRIORITY   = 24; // near max, timing critical
static const UBaseType_t PROCESSING_TASK_PRIORITY = 5;
static const uint32_t SAMPLING_TASK_STACK    = 4096;
static const uint32_t PROCESSING_TASK_STACK  = 8192;

// ---------------------------------------------------------------------
// WiFi Access Point - device creates its own hotspot, no router needed.
// Connect your phone to this network, then browse to 192.168.4.1
// ---------------------------------------------------------------------
static const char* const WIFI_AP_SSID     = "Vibration DAQ";
static const char* const WIFI_AP_PASSWORD = "00000000"; // >= 8 chars, or "" for open

// ---------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------
static const uint16_t CALIBRATION_SAMPLES = 500; // ~0.5s stationary at startup

#endif  // CONFIG_H_
