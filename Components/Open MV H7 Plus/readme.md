# Open Mv H7 Plus
https://github.com/user-attachments/assets/e64f9f64-35ea-4ba6-b662-329799734876

## General Overview
The OpenMV Cam H7 Plus is a compact, high-performance microcontroller board designed for real-world machine vision applications. It integrates a powerful STM32H743XI microcontroller with a high-resolution, interchangeable camera module. The system is programmed in MicroPython, enabling rapid development of complex vision tasks that are executed directly on the camera, functioning as a dedicated vision co-processor.

## Utilities
- Real-time color blob tracking and analysis (area, centroid, orientation).
- Fiducial marker detection (aprilTags, QR Codes) for navigation and localization.
- Execution of TensorFlow Lite for Microcontrollers (TFLM) models for on-device object detection and classification.
- Feature detection/matching (ORB, SIFT) and template matching.
- Optical Flow and image registration for motion analysis.

## Electronics
- MCU: STM32H743XI ARM Cortex M7, clocked at 480 MHz, with 1MB SRAM and 2MB Flash.
- Camera Sensor: OV5640, 5MP (2592x1944), with an interchangeable lens mount.
- Operation: The system boots and runs a main.py script stored on its internal filesystem or a microSD card. In a typical loop, it captures an image frame into RAM, executes a user-defined vision algorithm on it, and outputs structured results (e.g., coordinates, classes, IDs) via a serial interface like UART, I²C, or SPI.

## How to Program It
Programming is done in MicroPython using the OpenMV IDE. The code runs on the camera itself. The C++ code would run on the receiving microcontroller.

```py
import sensor, time, image
from pyb import UART

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
red_threshold = (30, 100, 15, 127, 15, 127) # L*a*b*
uart = UART(3, 115200) # UART3 on pins P4 (TX), P5 (RX)

while(True):
    img = sensor.snapshot()
    blobs = img.find_blobs([red_threshold])
    if blobs:
        b = blobs[0]
        # Format: "B,cx,cy\n"
        uart.write(f"B,{b.cx()},{b.cy()}\n")
```
## Connection to Arduino NANO ESP32
- VIN: Connect to 5V on the Nano ESP32.
- GND: Connect to GND on the Nano ESP32 (common ground).
- P4 (UART3 TX): Connect to a hardware serial RX pin on the Nano ESP32.
- P5 (UART3 RX): Connect to a hardware serial TX pin on the Nano ESP32.

## Operating Voltage
- Input Voltage (VIN / VUSB): 3.6V to 5V DC.
- I/O Logic Level: 3.3V (most I/O pins are 5V tolerant).

# End Page
Seteki 2025 - Components - Open MV H7 Plus

