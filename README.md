# LTA Fly-By-Wire Control System

A CircuitPython implementation of a lighter-than-air vehicle control system featuring dual tiltrotor propulsion with servo-actuated thrust vectoring.

## Hardware Configuration
- SCL: I2C clock
- SDA: I2C data
- D5: Rotational command pulse input
- D6: Forward command pulse input
- D9: Left fan PWM output
- D10: Right fan PWM output
- D13: BNO055 calibration LED indicator

## Control Parameters
- Operating frequency: 40Hz
- PID Control Values:
  - Kp = 0.5
  - Ki = 0.1
  - Kd = 14
- Altitude PID Values:
  - Kp_Alt = 3
  - Ki_Alt = 1
  - Kd_Alt = 10

## Dependencies
- CircuitPython
- adafruit_bno055
- adafruit_bmp3xx
- adafruit_pca9685
- pulseio
- time
- board
- busio

## Hardware Components
- BNO055 IMU
- PCA9685 PWM Controller
- BMP3XX Altitude Sensor
- Dual Servo Motors
- Three Ducted Fans

## Features
- Real-time orientation control
- Altitude hold capability
- Servo-controlled thrust vectoring
- Wind disturbance rejection
- Integrated PWM signal processing

## Notes
Do not confuse simulator files with actual CircuitPython hardware files. Use appropriate libraries for hardware implementation.
