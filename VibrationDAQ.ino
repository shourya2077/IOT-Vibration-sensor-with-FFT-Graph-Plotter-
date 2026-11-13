/*
 * VibrationDAQ.ino
 *
 * Portable vibration DAQ: put the ESP32 + MPU6500 on any machine,
 * robotic arm, motor, etc. It streams a live FFT (plus RMS / dominant
 * frequency per axis) to a phone browser over WiFi -- no app needed,
 * no internet needed.
 *
 * ARCHITECTURE (why it's fast):
 *  - A hardware timer (esp_timer) ticks at SAMPLE_RATE_HZ and hands off
 *    to a dedicated SAMPLING TASK pinned to core 1, high priority.
 *    That task ONLY talks to the I2C sensor and pushes into a buffer --
 *    nothing else competes with it for core 1, so sample timing stays
 *    tight regardless of what WiFi/FFT/JSON work is happening.
 *  - Two VibrationBuffer instances are passed back and forth between the
 *    sampling task and a PROCESSING TASK (core 0) through a pair of
 *    FreeRTOS queues (classic double-buffer / producer-consumer pattern).
 *    No mutex needed on the hot path.
 *  - The processing task runs the FFT (per axis) and pushes the result
 *    out over WebSocket. It shares core 0 with the WiFi/AsyncTCP stack,
 *    which is fine since it only needs to run a few times per second.
 *
 * WIRING (ESP32 DevKit V1, 30-pin  <->  MPU6500):
 *   MPU6500 VCC -> ESP32 3V3   (power sensor FROM the ESP32's own 3V3
 *                                pin, not a separate supply rail --
 *                                mixing supplies is a classic source of
 *                                flaky I2C / ghost devices)
 *   MPU6500 GND -> ESP32 GND
 *   MPU6500 SCL -> GPIO22
 *   MPU6500 SDA -> GPIO21
 *   MPU6500 AD0 -> GND        (selects I2C address 0x68)
 *
 * SETUP:
 *   1. Install libraries (Library Manager or manually):
 *        - bolderflight/invensense-imu
 *        - kosme/arduinoFFT (v2.x)
 *        - ESP32Async/ESPAsyncWebServer  + ESP32Async/AsyncTCP
 *        - ArduinoJson (v7)
 *   2. Upload the contents of the /data folder (dashboard.html) to
 *      LittleFS using the "Sketch Data Upload" tool (see README.md).
 *   3. Flash this sketch.
 *   4. On your phone, join WiFi network "VibrationDAQ" (see config.h
 *      for password), then open http://192.168.4.1 in the browser.
 *   5. Keep the device still for ~1s after power-on while it calibrates.
 */

#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

#include "config.h"
#include "MPU6500Sensor.h"
#include "VibrationBuffer.h"
#include "FFTProcessor.h"
#include "TelemetryServer.h"

// ---------------------------------------------------------------------
// Global objects
// ---------------------------------------------------------------------
MPU6500Sensor sensor(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);
TelemetryServer telemetry(WIFI_AP_SSID, WIFI_AP_PASSWORD);

VibrationBuffer bufferA(FFT_SIZE);
VibrationBuffer bufferB(FFT_SIZE);

static QueueHandle_t emptyQueue;    // holds buffers ready to be filled
static QueueHandle_t filledQueue;   // holds buffers ready to be processed
static SemaphoreHandle_t sampleSemaphore;
static esp_timer_handle_t sampleTimer;

static TaskHandle_t samplingTaskHandle;
static TaskHandle_t processingTaskHandle;

// ---------------------------------------------------------------------
// Timer callback: dispatched in the esp_timer task context (not a true
// ISR), so a plain semaphore give is safe here.
// ---------------------------------------------------------------------
static void onSampleTimer(void* /*arg*/) {
  xSemaphoreGive(sampleSemaphore);
}

// ---------------------------------------------------------------------
// Core 1: sampling task. Precise timing, nothing else runs here.
// ---------------------------------------------------------------------
static void samplingTask(void* /*pvParameters*/) {
  VibrationBuffer* current = nullptr;
  xQueueReceive(emptyQueue, &current, portMAX_DELAY);
  current->reset();

  for (;;) {
    xSemaphoreTake(sampleSemaphore, portMAX_DELAY);

    float x, y, z;
    if (sensor.read(x, y, z)) {
      if (current->push(x, y, z)) {
        // Buffer just filled up -- hand it to the processing task and
        // grab a fresh one to keep sampling without a gap.
        xQueueSend(filledQueue, &current, portMAX_DELAY);
        xQueueReceive(emptyQueue, &current, portMAX_DELAY);
        current->reset();
      }
    }

    // Checked once per sample tick (cheap: a volatile bool read). If the
    // dashboard just sent "TARE", recalibrate now. This blocks sampling
    // for ~0.5s -- an expected, user-triggered blip, not a steady-state
    // cost.
    if (telemetry.consumeTareRequest()) {
      sensor.calibrate(CALIBRATION_SAMPLES);
    }
  }
}

// ---------------------------------------------------------------------
// Core 0: processing task. FFT + JSON + WebSocket send.
// ---------------------------------------------------------------------
static void processingTask(void* /*pvParameters*/) {
  FFTProcessor fftProc(FFT_SIZE, SAMPLE_RATE_HZ);
  const uint16_t bins = fftProc.binCount();

  AxisFFTResult resX, resY, resZ;
  resX.magnitude = new float[bins];
  resY.magnitude = new float[bins];
  resZ.magnitude = new float[bins];

  VibrationBuffer* ready = nullptr;
  for (;;) {
    xQueueReceive(filledQueue, &ready, portMAX_DELAY);

    fftProc.processAxis(ready->x(), resX);
    fftProc.processAxis(ready->y(), resY);
    fftProc.processAxis(ready->z(), resZ);

    telemetry.broadcastFFT(millis(), SAMPLE_RATE_HZ, FFT_SIZE, bins,
                            resX, resY, resZ);

    xQueueSend(emptyQueue, &ready, portMAX_DELAY);
  }
}

// ---------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Vibration DAQ ===");

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(I2C_CLOCK_HZ);

  Serial.println("Initializing MPU6500...");
  if (!sensor.begin()) {
    Serial.println("FAILED to init MPU6500. Check SDA/SCL/AD0/3V3/GND wiring.");
    while (true) delay(1000);
  }

  Serial.println("Calibrating -- keep the device still...");
  if (!sensor.calibrate(CALIBRATION_SAMPLES)) {
    Serial.println("Calibration timed out. Check sensor connection.");
    while (true) delay(1000);
  }
  Serial.println("Calibration OK.");

  Serial.println("Starting WiFi AP + web server...");
  if (!telemetry.begin()) {
    Serial.println("FAILED to start telemetry server (LittleFS not mounted, "
                    "or dashboard.html not uploaded -- see README.md).");
    while (true) delay(1000);
  }
  Serial.print("Connect your phone to WiFi: ");
  Serial.println(WIFI_AP_SSID);
  Serial.println("Then open http://192.168.4.1 in a browser.");

  // Double-buffer queues
  emptyQueue = xQueueCreate(2, sizeof(VibrationBuffer*));
  filledQueue = xQueueCreate(2, sizeof(VibrationBuffer*));
  VibrationBuffer* a = &bufferA;
  VibrationBuffer* b = &bufferB;
  xQueueSend(emptyQueue, &a, 0);
  xQueueSend(emptyQueue, &b, 0);

  sampleSemaphore = xSemaphoreCreateBinary();

  xTaskCreatePinnedToCore(samplingTask, "Sampling", SAMPLING_TASK_STACK,
                           nullptr, SAMPLING_TASK_PRIORITY,
                           &samplingTaskHandle, SAMPLING_TASK_CORE);
  xTaskCreatePinnedToCore(processingTask, "Processing", PROCESSING_TASK_STACK,
                           nullptr, PROCESSING_TASK_PRIORITY,
                           &processingTaskHandle, PROCESSING_TASK_CORE);

  esp_timer_create_args_t timerArgs = {};
  timerArgs.callback = &onSampleTimer;
  timerArgs.arg = nullptr;
  timerArgs.dispatch_method = ESP_TIMER_TASK;
  timerArgs.name = "sample_timer";
  esp_timer_create(&timerArgs, &sampleTimer);
  esp_timer_start_periodic(sampleTimer, 1000000UL / SAMPLE_RATE_HZ);

  Serial.println("Running.");
}

void loop() {
  // All real work happens in the two pinned tasks above. This just
  // prints a periodic heartbeat so you can confirm it's alive over
  // Serial while bench-testing.
  static uint32_t lastLog = 0;
  if (millis() - lastLog > 5000) {
    lastLog = millis();
    Serial.printf("[heartbeat] clients connected: %u\n",
                   static_cast<unsigned>(telemetry.connectedClients()));
  }
  delay(1000);
}
