# BGT60LTR11AIP-Target-speed-calculation

This repository provides an implementation for calculating the target speed from the radar data of the **BGT60LTR11AIP** sensor. It processes FFT spectrum data to determine the speed of a detected object, distinguishing between approaching and departing targets based on Doppler frequency.

## Features

- **Peak Frequency Detection**: Identifies the peak frequency from the FFT spectrum.
- **Target Speed Calculation**: Computes the target's speed in real-time based on Doppler shift.
- **Motion Direction Detection**: Determines whether the target is approaching or departing from the radar.
- **Scalable Parameters**: Allows easy customization of sampling rate, FFT size, and other parameters.

## Requirements

To use or integrate this application, you need:
- **Hardware**: 
  - BGT60LTR11AIP radar module.
  - STM32H750VBT.
- **Software**:
  - STM32CubeIDE, CMSIS DSP Libray.
  - Optional: MATLAB or Python for visualization and testing.


