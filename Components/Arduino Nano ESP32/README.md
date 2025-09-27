# Arduino NANO ESP32

![Add a subheading (16 x 5 cm) (4096 x 2160 px) (A4 (Landscape)) (5)](https://github.com/user-attachments/assets/c4ba0f49-8546-4601-80e8-c29e0564de42)

## General Overview

The Arduino Nano `ESP32` is a microcontroller development board that integrates the powerful Espressif` ESP32-S3 System-on-Chip (SoC)` into the compact Arduino Nano form factor. It combines a high-performance dual-core processor with `integrated Wi-Fi 802.11b/g/n` and Bluetooth 5 LE, making it ideal for advanced `IoT` and connected robotics projects.

## Utilities
- ioT Endpoints: Wireless sensor nodes that collect data and transmit it via Wi-Fi or Bluetooth.
- Robotics Control Hub: Serves as the central brain for robots, managing motor drivers, sensors, and wireless communication.
- Human-Machine Interface (HMI): Can host web servers or connect to mobile apps for remote control and monitoring.
- Low-Power Systems: Utilizes the ESP32-S3's deep sleep modes for battery-powered applications.

## Electronics
- SoC: Espressif ESP32-S3 WROOM-1 module.
- CPU: Dual-core 32-bit Xtensa LX7 microprocessor, clocked up to 240 MHz.
- Wireless: Integrated 2.4 GHz Wi-Fi and Bluetooth 5 LE radio.
- Memory: 512 KB SRAM, 384 KB ROM, 16 KB SRAM in RTC.
- Operation: The board executes user-compiled code uploaded to its internal flash memory. The code directly manipulates the General Purpose Input/Output (GPIO) pins to interface with external hardware (sensors, actuators) and utilizes the onboard radio for wireless networking and communication protocols. An onboard USB-to-UART converter handles programming and serial communication.

## Code
Programming is typically done in C++ via the Arduino IDE or in Python via MicroPython/CircuitPython firmware.

### `C++`
```ino
#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  WiFi.begin("YourSSID", "YourPassword");
  while (WiFi.status() != WL_CONNECTED) { delay(500); }
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}
```

### `Micropython`
```py

import machine
import time
import network

led = machine.Pin("LED", machine.Pin.OUT)
sta_if = network.WLAN(network.STA_IF); sta_if.active(True)
sta_if.connect("YourSSID", "YourPassword")

while not sta_if.isconnected():
    pass

while True:
    led.toggle()
    time.sleep(0.5)

```

## Operative voltage
Logic Level: 3.3V. GPIO pins are not 5V tolerant.
Input Voltage (VIN): 5V to 21V DC. An onboard regulator steps this down to 3.3V.

# End Page
Seteki 2025 - Components - Arduino NANO ESP32
