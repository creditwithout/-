# Sparkfun BNO086
![Add a subheading (16 x 5 cm) (4096 x 2160 px) (A4 (Landscape)) (4)](https://github.com/user-attachments/assets/64636271-545e-4edc-9b0d-b9e4e7648fa6)

## General Overview
The BNO085/BNO086 is a System-in-Package (SiP) that integrates a 3-axis accelerometer, 3-axis gyroscope, 3-axis magnetometer, and a 32-bit ARM Cortex M0+ microcontroller. Its defining feature is the onboard CEVA SH-2 firmware, which performs high-level sensor fusion, offloading complex calculations from the host MCU and providing drift-corrected, stable orientation vectors.

## Electronics and How It Works

- The three internal sensors (accelerometer, gyroscope, magnetometer) continuously gather raw motion and magnetic field data.
- This raw data is fed directly to the internal ARM M0+ processor.
- The processor executes proprietary fusion algorithms (SH-2) that combine the sensor inputs. The algorithm uses the accelerometer and magnetometer as stable, long-term references to correct the short-term drift inherent in the gyroscope.
- The output is not raw data but processed, fused vectors. The most useful is the Game Rotation Vector, which provides a quaternion-based orientation that is gravity-compensated and corrected for magnetic interference, yielding a highly accurate yaw measurement.

## How to Program It
Communication is via `I²C or UART`. Programming involves initializing the sensor hub and requesting specific sensor reports. The host MCU then polls the device for available data packets.

### `C++ (Arduino Framework)`
```ino
#include <SparkFun_BNO08x_Arduino_Library.h>
BNO08x myIMU;

void setup() {
  Wire.begin();
  myIMU.begin();
  // Request the report that provides stable Yaw
  myIMU.enableGameRotationVector(10000); // 10ms update interval
}

void loop() {
  if (myIMU.dataAvailable() == true) {
    float yaw = myIMU.getYaw(); // Read fused and corrected yaw in degrees
  }
}
```

### `Python`
```py
import board
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_GAME_ROTATION_VECTOR

i2c = board.I2C()
bno = BNO08X_I2C(i2c)
bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)

while True:
    # get_quat() provides the most stable orientation data
    quat_i, quat_j, quat_k, quat_real = bno.quaternion
    # Further math is needed to convert quaternion to Euler angles (yaw)
```
# End Page
Seteki 2025 - Components - BNO086
