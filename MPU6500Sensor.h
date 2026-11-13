/*
 * MPU6500Sensor.h
 *
 * Thin OOP wrapper around bfs::Mpu6500 (bolderflight/invensense-imu).
 * Adds gravity-bias calibration so the FFT only sees oscillatory
 * (vibration) content, regardless of how the device is mounted/oriented
 * on the machine surface.
 *
 * Verified against bolderflight/invensense-imu src/mpu6500.h directly.
 * If you're on a different library version and something doesn't
 * compile, check the enum names in your installed mpu6500.h first --
 * same rule of thumb as last time with the MPU9250 port.
 */
#ifndef MPU6500_SENSOR_H_
#define MPU6500_SENSOR_H_

#include <Arduino.h>
#include <Wire.h>
#include "mpu6500.h"

class MPU6500Sensor {
 public:
  MPU6500Sensor(TwoWire* wire, bfs::Mpu6500::I2cAddr addr);

  // Initializes the bus + sensor, applies range/DLPF/rate config.
  // Returns false if the sensor doesn't respond (check wiring: SDA/SCL,
  // AD0->GND, 3V3, GND).
  bool begin();

  // Averages CALIBRATION_SAMPLES readings while the device is assumed
  // stationary and stores the result as the zero-vibration bias.
  // Call this at boot, and again ("tare") whenever the device is moved
  // to a new surface/orientation.
  bool calibrate(uint16_t numSamples);

  // Blocking-ish poll of the sensor. Returns true if a fresh sample was
  // available and out_x/y/z were updated (bias already subtracted,
  // units m/s^2). Returns false if no new data yet (call again).
  bool read(float& out_x, float& out_y, float& out_z);

  bool isCalibrated() const { return calibrated_; }

 private:
  bfs::Mpu6500 imu_;
  TwoWire* wire_;
  bfs::Mpu6500::I2cAddr addr_;
  float biasX_, biasY_, biasZ_;
  bool calibrated_;
};

#endif  // MPU6500_SENSOR_H_
