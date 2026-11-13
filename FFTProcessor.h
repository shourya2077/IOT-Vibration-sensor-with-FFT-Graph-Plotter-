/*
 * FFTProcessor.h
 *
 * Wraps arduinoFFT (v2.x API, verified against kosme/arduinoFFT source)
 * to turn a raw time-domain buffer into a magnitude spectrum, plus a
 * couple of scalar summary numbers (RMS, dominant frequency/magnitude)
 * that are cheap to compute and very useful on the dashboard.
 */
#ifndef FFT_PROCESSOR_H_
#define FFT_PROCESSOR_H_

#include <Arduino.h>
#include <arduinoFFT.h>

struct AxisFFTResult {
  float* magnitude;   // length = fftSize/2, caller-owned, filled in place
  float rms;          // time-domain RMS (m/s^2), DC/bias already removed
  float peakFreqHz;
  float peakMagnitude;
};

class FFTProcessor {
 public:
  FFTProcessor(uint16_t fftSize, float sampleRateHz);
  ~FFTProcessor();

  // raw* must each point to fftSize samples. out.magnitude must already
  // point at a caller-allocated buffer of length fftSize/2.
  void processAxis(const float* raw, AxisFFTResult& out);

  uint16_t binCount() const { return fftSize_ / 2; }
  float frequencyResolutionHz() const { return sampleRateHz_ / fftSize_; }

 private:
  // Declaration order matters here: vReal_/vImag_ must be constructed
  // before fft_, since fft_'s constructor takes pointers to them.
  uint16_t fftSize_;
  float sampleRateHz_;
  double* vReal_;
  double* vImag_;
  ArduinoFFT<double> fft_;
};

#endif  // FFT_PROCESSOR_H_
