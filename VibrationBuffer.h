/*
 * VibrationBuffer.h
 *
 * A fixed-size, 3-axis sample buffer. Two instances of this class are
 * handed back and forth between the sampling task and the processing
 * task through a pair of FreeRTOS queues (see VibrationDAQ.ino), so
 * neither task ever touches memory the other one is using -- no mutex
 * needed on the hot path, which is what keeps the sampling loop jitter-free.
 */
#ifndef VIBRATION_BUFFER_H_
#define VIBRATION_BUFFER_H_

#include <Arduino.h>

class VibrationBuffer {
 public:
  explicit VibrationBuffer(uint16_t capacity)
      : capacity_(capacity), count_(0) {
    x_ = new float[capacity_];
    y_ = new float[capacity_];
    z_ = new float[capacity_];
  }

  ~VibrationBuffer() {
    delete[] x_;
    delete[] y_;
    delete[] z_;
  }

  inline void reset() { count_ = 0; }

  // Returns true once the buffer becomes full as a result of this push.
  inline bool push(float x, float y, float z) {
    if (count_ >= capacity_) return true; // already full, caller should have swapped
    x_[count_] = x;
    y_[count_] = y;
    z_[count_] = z;
    count_++;
    return count_ >= capacity_;
  }

  inline bool isFull() const { return count_ >= capacity_; }
  inline uint16_t capacity() const { return capacity_; }

  inline float* x() { return x_; }
  inline float* y() { return y_; }
  inline float* z() { return z_; }

 private:
  uint16_t capacity_;
  uint16_t count_;
  float* x_;
  float* y_;
  float* z_;
};

#endif  // VIBRATION_BUFFER_H_
