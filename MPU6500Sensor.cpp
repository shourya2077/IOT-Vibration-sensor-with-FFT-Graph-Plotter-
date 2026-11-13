#include "MPU6500Sensor.h"
#include "config.h"

MPU6500Sensor::MPU6500Sensor(TwoWire* wire, bfs::Mpu6500::I2cAddr addr)
    : wire_(wire), addr_(addr),
      biasX_(0), biasY_(0), biasZ_(0),
      calibrated_(false) {}

bool MPU6500Sensor::begin() {
  // Matches the library author's own documented setup pattern
  // (Config() then Begin()) rather than the alternate constructor.
  imu_.Config(wire_, addr_);
  if (!imu_.Begin()) {
    return false;
  }
  // +-8g gives good headroom for machine/robotic-arm vibration bursts
  // without clipping, while still keeping decent resolution.
  if (!imu_.ConfigAccelRange(bfs::Mpu6500::ACCEL_RANGE_8G)) {
    return false;
  }
  // Widest DLPF this driver exposes -- see the note in config.h about
  // effective usable bandwidth.
  if (!imu_.ConfigDlpfBandwidth(bfs::Mpu6500::DLPF_BANDWIDTH_184HZ)) {
    return false;
  }
  // SRD=0 -> output rate = 1000Hz / (1 + 0) = 1000Hz, matching
  // SAMPLE_RATE_HZ in config.h. If you change SAMPLE_RATE_HZ, update
  // this divider too: srd = (1000 / SAMPLE_RATE_HZ) - 1.
  if (!imu_.ConfigSrd(0)) {
    return false;
  }
  return true;
}

bool MPU6500Sensor::calibrate(uint16_t numSamples) {
  double sumX = 0, sumY = 0, sumZ = 0;
  uint16_t got = 0;
  uint32_t startMs = millis();
  const uint32_t timeoutMs = 5000; // don't hang forever if wiring is bad

  while (got < numSamples) {
    if (imu_.Read() && imu_.new_imu_data()) {
      sumX += imu_.accel_x_mps2();
      sumY += imu_.accel_y_mps2();
      sumZ += imu_.accel_z_mps2();
      got++;
    }
    if (millis() - startMs > timeoutMs) {
      calibrated_ = false;
      return false;
    }
  }

  biasX_ = static_cast<float>(sumX / numSamples);
  biasY_ = static_cast<float>(sumY / numSamples);
  biasZ_ = static_cast<float>(sumZ / numSamples);
  calibrated_ = true;
  return true;
}

bool MPU6500Sensor::read(float& out_x, float& out_y, float& out_z) {
  if (!imu_.Read() || !imu_.new_imu_data()) {
    return false;
  }
  out_x = imu_.accel_x_mps2() - biasX_;
  out_y = imu_.accel_y_mps2() - biasY_;
  out_z = imu_.accel_z_mps2() - biasZ_;
  return true;
}
