#include "TelemetryServer.h"
#include <WiFi.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <math.h>
#include <string.h>

namespace {
inline float round2(float v) { return roundf(v * 100.0f) / 100.0f; }
}  // namespace

TelemetryServer::TelemetryServer(const char* apSsid, const char* apPassword)
    : ssid_(apSsid),
      password_(apPassword),
      server_(80),
      ws_("/ws"),
      tareRequested_(false) {}

bool TelemetryServer::begin() {
  if (!LittleFS.begin(true)) {
    // true = format on mount failure. If this keeps failing, the
    // dashboard.html file was probably never uploaded to LittleFS --
    // see the README for the "Sketch Data Upload" step.
    return false;
  }

  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(ssid_, (strlen(password_) >= 8) ? password_ : nullptr)) {
    return false;
  }

  ws_.onEvent([this](AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
    this->handleWsEvent(server, client, type, arg, data, len);
  });
  server_.addHandler(&ws_);

  server_.serveStatic("/", LittleFS, "/").setDefaultFile("dashboard.html");

  server_.onNotFound([](AsyncWebServerRequest* request) {
    request->send(404, "text/plain", "Not found");
  });

  server_.begin();
  return true;
}

void TelemetryServer::handleWsEvent(AsyncWebSocket* /*server*/,
                                     AsyncWebSocketClient* /*client*/,
                                     AwsEventType type, void* /*arg*/,
                                     uint8_t* data, size_t len) {
  if (type == WS_EVT_DATA) {
    // Simple text commands from the dashboard. Currently only "TARE" is
    // recognized; add more `else if` branches here as the dashboard grows.
    String msg;
    msg.reserve(len);
    for (size_t i = 0; i < len; i++) msg += static_cast<char>(data[i]);
    if (msg == "TARE") {
      tareRequested_ = true;
    }
  }
}

bool TelemetryServer::consumeTareRequest() {
  if (tareRequested_) {
    tareRequested_ = false;
    return true;
  }
  return false;
}

void TelemetryServer::broadcastFFT(uint32_t timestampMs, float sampleRateHz,
                                    uint16_t fftSize, uint16_t bins,
                                    const AxisFFTResult& x,
                                    const AxisFFTResult& y,
                                    const AxisFFTResult& z) {
  if (ws_.count() == 0) return;  // nobody listening, skip the work entirely

  JsonDocument doc;
  doc["t"] = timestampMs;
  doc["fs"] = sampleRateHz;
  doc["n"] = fftSize;
  doc["df"] = round2(sampleRateHz / fftSize);

  const AxisFFTResult* axes[3] = {&x, &y, &z};
  const char* names[3] = {"x", "y", "z"};
  JsonObject axesObj = doc["axes"].to<JsonObject>();
  for (int a = 0; a < 3; a++) {
    JsonObject axisObj = axesObj[names[a]].to<JsonObject>();
    axisObj["rms"] = round2(axes[a]->rms);
    axisObj["peakF"] = round2(axes[a]->peakFreqHz);
    axisObj["peakM"] = round2(axes[a]->peakMagnitude);
    JsonArray mag = axisObj["mag"].to<JsonArray>();
    for (uint16_t i = 0; i < bins; i++) {
      mag.add(round2(axes[a]->magnitude[i]));
    }
  }

  size_t len = measureJson(doc);
  AsyncWebSocketMessageBuffer* buffer = ws_.makeBuffer(len);
  if (!buffer) return;  // allocation failed (low memory) -- drop this frame
  serializeJson(doc, reinterpret_cast<char*>(buffer->get()), len);
  ws_.textAll(buffer);
}
