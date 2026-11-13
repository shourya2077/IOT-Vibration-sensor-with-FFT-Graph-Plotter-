# Vibration DAQ — ESP32 + MPU6500

Portable vibration analyzer. Stick it to a machine, motor housing, or
robotic-arm joint, and it streams a live 3-axis FFT + RMS + dominant
frequency to your phone's browser over the ESP32's own WiFi hotspot.
No app install, no internet connection needed, no router needed.

## What's in this folder

```
VibrationDAQ/
├── VibrationDAQ.ino        <- main sketch (setup/loop, FreeRTOS tasks)
├── config.h                <- all the tunable constants, start here
├── MPU6500Sensor.h/.cpp    <- sensor class (bias calibration + reads)
├── VibrationBuffer.h       <- double-buffer sample container
├── FFTProcessor.h/.cpp     <- windowing + FFT + RMS + peak detection
├── TelemetryServer.h/.cpp  <- WiFi AP + WebSocket + JSON streaming
├── data/
│   └── dashboard.html      <- the phone-side UI (uploaded to LittleFS)
└── README.md                <- this file
```

Every class is deliberately self-contained (sensor / FFT / networking
are three separate objects that only talk through plain structs), so
you can swap or extend any one piece later without touching the others
— e.g. add a second sensor, switch the transport to binary frames, or
add a waveform view to the dashboard.

## Wiring

```
   MPU6500              ESP32 DevKit V1 (30-pin)
  ┌────────┐            ┌──────────────────┐
  │  VCC   ├────────────┤ 3V3              │
  │  GND   ├────────────┤ GND              │
  │  SCL   ├────────────┤ GPIO22           │
  │  SDA   ├────────────┤ GPIO21           │
  │  AD0   ├──── GND    │                  │
  └────────┘            └──────────────────┘
```

Power the MPU6500 **from the ESP32's own 3V3 pin**, not a separate
supply rail. Mixed supplies with a shared-but-not-quite-common ground
is a classic way to get flaky I2C reads and "ghost" devices on the bus.

## Libraries to install

Arduino IDE → Library Manager (or manual install where noted):

| Library | Notes |
|---|---|
| `bolderflight/invensense-imu` | Same one used for your rocket flight computer — this project uses the `Mpu6500` class from it directly. |
| `arduinoFFT` (kosme) | v2.x. This project uses the v2 API (`ArduinoFFT<double>`, `FFTWindow::Hann`). |
| `ESPAsyncWebServer` **+** `AsyncTCP` | Install the **ESP32Async** org forks specifically: `ESP32Async/ESPAsyncWebServer` and `ESP32Async/AsyncTCP`. The original `me-no-dev` repo is stale and has known problems on ESP32 Arduino core 3.x — same family of issue as the watchdog API change you hit before. |
| `ArduinoJson` | v7.x. |

If you're on PlatformIO instead, `lib_deps`:
```ini
lib_deps =
    bolderflight/invensense-imu
    kosme/arduinoFFT @ ^2.0
    ESP32Async/ESPAsyncWebServer
    ESP32Async/AsyncTCP
    bblanchon/ArduinoJson @ ^7
```

## Uploading the dashboard (LittleFS)

The HTML/CSS/JS dashboard lives on the ESP32's flash filesystem, separate
from the firmware. This means **you can redesign or upgrade the
dashboard later by just re-uploading `dashboard.html` — no recompiling
or reflashing the firmware.**

Arduino IDE 2.x doesn't bundle a filesystem uploader by default, so:

1. Install the **"Arduino LittleFS Upload"** plugin (`earlephilhower/arduino-littlefs-upload`) — in Arduino IDE 2.x this goes in the IDE's plugin manner per that repo's README (search "arduino littlefs upload" for the current install steps, it changes occasionally between IDE versions).
2. Once installed, `Ctrl+Shift+P` (or `Cmd+Shift+P`) → "Upload LittleFS to Pico/ESP32/ESP8266" — this uploads everything in the `data/` folder (just `dashboard.html` here) to the ESP32's flash.
3. Then upload the sketch itself as normal.

If the filesystem step ever gives you trouble and you just want to get
running: you can instead paste `dashboard.html`'s contents into a raw
string in a header file and `server.send_P()` it directly from
`TelemetryServer::begin()` — trades the "edit HTML without reflashing"
convenience for a simpler one-step upload. Worth knowing about as a
fallback, but LittleFS is the better setup for iterating on the UI.

## First run

1. Flash the sketch, upload the filesystem, power the board.
2. Open the Serial Monitor at 115200 baud — it'll walk through sensor
   init, calibration, and WiFi AP startup, and tell you if anything
   failed (and what to check).
3. **Keep the device still** for about half a second right after
   power-on — that's when it samples a baseline to subtract gravity/bias
   from every axis, so the FFT only sees actual vibration, not the
   static 1g offset (this works regardless of mounting orientation).
4. On your phone: join WiFi network `VibrationDAQ` (password in
   `config.h`), then open `http://192.168.4.1`.
5. Whenever you move the device to a different surface or orientation,
   hit **Tare / Recalibrate** on the dashboard before trusting the
   readings.

## Tuning

Everything's in `config.h`:

- `FFT_SIZE` (default 256): bigger = finer frequency resolution, slower
  refresh rate. Must stay a power of 2.
- `SAMPLE_RATE_HZ` (default 1000): if you change it, also update the
  `ConfigSrd()` call in `MPU6500Sensor::begin()` — the comment right
  above it has the formula.
- `WIFI_AP_SSID` / `WIFI_AP_PASSWORD`.

Refresh rate ≈ `SAMPLE_RATE_HZ / FFT_SIZE` — the defaults give you
~3.9 FFT updates per second, which reads as "live" without hammering
the WiFi link.

## Important limitation — please read before trusting this for fault diagnosis

The MPU6500's onboard low-pass filter only exposes bandwidths up to
**184 Hz** through this driver (no "wide open" mode). So although the
FFT technically plots out to `SAMPLE_RATE_HZ / 2` (500 Hz with the
defaults), content above roughly 180 Hz is already attenuated by the
sensor itself before you ever see it. That's genuinely fine for the
things this class of sensor is good at — **imbalance, looseness,
misalignment, and general "something's shaking that shouldn't be"**
signatures, which mostly live under 100–150 Hz for typical rotating
machinery. It is **not** a substitute for a real piezo vibration
accelerometer if you need high-frequency **bearing-defect envelope
analysis**, which typically needs a flat response well into the kHz
range. Know which job you're asking it to do.

## Extending this later

The architecture was built with your "upgrade the dashboard later"
requirement in mind:

- **New dashboard views** (waveform strip, waterfall/spectrogram,
  alarm thresholds, data logging): add a JS class next to `FFTChart`
  in `dashboard.html` and feed it from `Dashboard._onData()`. No
  firmware changes needed unless the new view needs new data.
- **New fields in the data stream**: add a key in
  `TelemetryServer::broadcastFFT()` (e.g. a raw waveform array,
  gyro data, temperature) — JSON makes this an additive change on
  both ends.
- **Binary protocol for even more speed**: if JSON serialization ever
  becomes the bottleneck (unlikely at ~4 Hz), `TelemetryServer` is the
  only class that would need to change — everything upstream of it
  (`FFTProcessor`, `MPU6500Sensor`) is transport-agnostic.
