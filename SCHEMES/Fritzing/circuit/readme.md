# Fritzing circuit
<img width="1020" height="737" alt="image" src="https://github.com/user-attachments/assets/b1418137-58da-46c3-a822-cb12e5cafe46" />

## Components (briefly)
- `BNO086` The SparkFun BNO086 is a 9-DoF IMU sensor with integrated sensor fusion via the Hillcrest SH-2 firmware. It combines a 3-axis accelerometer, gyroscope, and magnetometer, producing orientation in quaternions for minimal singularities. The sensor’s core output is the unit quaternion vector q = [q_w, q_x, q_y, q_z].

> [!IMPORTANT]
> if you want more information of any component we use, you can click [Here](https://www.sparkfun.com/sparkfun-vr-imu-breakout-bno086-qwiic.html), here you'll find a mor extendend documentation in every component

- `OpenMV H7`: is a microcontroller-based machine vision board built around an `ARM Cortex-M7` MCU (e.g. 400 MHz in original H7) with double-precision FPU, ~1 MB SRAM + 2 MB flash memory. 
SparkFun
It provides interfaces (I²C, SPI, UART, etc.) to integrate additional sensors.

- Capacitors: we use capacitor as a temporary energy storage, charging when voltage rises and discharging when it drops, thus smoothing fluctuations.
In power supplies, they reduce ripple by maintaining voltage between peaks of rectified AC signals.
They act as low-impedance paths for high-frequency noise, shunting disturbances away from sensitive circuits.
The stored energy 
`𝑄=𝐶⋅𝑉`
provides instantaneous current to stabilize sudden load changes.
By filtering irregularities, they improve voltage regulation and protect components from transients.
Overall, capacitors stabilize DC rails and ensure consistent performance in electronic systems.
- `For comunicate the imu we use 12c and the camera uses UART.`
# End Page
Seteki 2025 - Schemes - Fritzing - Circuit
