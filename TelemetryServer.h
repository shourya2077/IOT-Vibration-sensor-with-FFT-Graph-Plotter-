/*
 * TelemetryServer.h
 *
 * Owns the WiFi Access Point, the LittleFS-served dashboard, and the
 * WebSocket that streams FFT results to the browser. Also listens for
 * simple text commands from the dashboard ("TARE") so the phone can
 * trigger a recalibration without needing a second communication path.
 *
 * Uses ESP32Async/ESPAsyncWebServer + ESP32Async/AsyncTCP -- the
 * actively-maintained fork, compatible with ESP32 Arduino core 3.x
 * (the original me-no-dev repo is stale and has known issues on core 3.x).
 */
#ifndef TELEMETRY_SERVER_H_
#define TELEMETRY_SERVER_H_

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include "FFTProcessor.h"

class TelemetryServer {
 public:
  TelemetryServer(const char* apSsid, const char* apPassword);

  // Mounts LittleFS, starts the SoftAP, sets up HTTP + WebSocket routes.
  // Returns false if LittleFS fails to mount (dashboard.html missing/not
  // uploaded) or the AP fails to start.
  bool begin();

  // Call once per FFT cycle from the processing task. bins = fftSize/2.
  void broadcastFFT(uint32_t timestampMs, float sampleRateHz,
                     uint16_t fftSize, uint16_t bins,
                     const AxisFFTResult& x,
                     const AxisFFTResult& y,
                     const AxisFFTResult& z);

  // Returns true exactly once after the dashboard sends a "TARE" command,
  // then clears itself. Poll this from the main loop or sampling setup.
  bool consumeTareRequest();

  size_t connectedClients() const { return ws_.count(); }

 private:
  void handleWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len);

  const char* ssid_;
  const char* password_;
  AsyncWebServer server_;
  AsyncWebSocket ws_;
  volatile bool tareRequested_;
};

#endif  // TELEMETRY_SERVER_H_
