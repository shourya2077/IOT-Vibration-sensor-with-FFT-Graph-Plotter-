#include "FFTProcessor.h"
#include <math.h>

FFTProcessor::FFTProcessor(uint16_t fftSize, float sampleRateHz)
    : fftSize_(fftSize),
      sampleRateHz_(sampleRateHz),
      vReal_(new double[fftSize]),
      vImag_(new double[fftSize]),
      fft_(vReal_, vImag_, fftSize, sampleRateHz) {}

FFTProcessor::~FFTProcessor() {
  delete[] vReal_;
  delete[] vImag_;
}

void FFTProcessor::processAxis(const float* raw, AxisFFTResult& out) {
  // --- RMS from the untouched time-domain signal (bias already removed
  // upstream by MPU6500Sensor) ---
  double sumSq = 0.0;
  for (uint16_t i = 0; i < fftSize_; i++) {
    sumSq += static_cast<double>(raw[i]) * static_cast<double>(raw[i]);
  }
  out.rms = static_cast<float>(sqrt(sumSq / fftSize_));

  // --- Load into the FFT working buffers ---
  for (uint16_t i = 0; i < fftSize_; i++) {
    vReal_[i] = static_cast<double>(raw[i]);
    vImag_[i] = 0.0;
  }

  fft_.windowing(FFTWindow::Hann, FFTDirection::Forward);
  fft_.compute(FFTDirection::Forward);
  fft_.complexToMagnitude();

  // Normalize to a single-sided amplitude spectrum (x2/N; Hann coherent
  // gain not separately corrected -- fine for relative/visual comparison,
  // which is what the dashboard is for).
  const uint16_t bins = fftSize_ / 2;
  const double norm = 2.0 / fftSize_;

  float peakMag = 0.0f;
  uint16_t peakIdx = 1; // skip bin 0 (DC) when looking for the dominant peak
  for (uint16_t i = 0; i < bins; i++) {
    float mag = static_cast<float>(vReal_[i] * norm);
    out.magnitude[i] = mag;
    if (i >= 1 && mag > peakMag) {
      peakMag = mag;
      peakIdx = i;
    }
  }

  out.peakMagnitude = peakMag;
  out.peakFreqHz = peakIdx * frequencyResolutionHz();
}
