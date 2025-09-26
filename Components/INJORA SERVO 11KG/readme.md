# INJORA 11KG SERVO

![Add a subheading (16 x 5 cm) (4096 x 2160 px) (A4 (Landscape)) (1)](https://github.com/user-attachments/assets/e7e9cacf-56f6-43d9-9656-88491bc41f40)

## 1. General Overview:
A servo motor is a self-contained, closed-loop electromechanical actuator. It comprises a DC motor, reduction gearbox, potentiometer for position feedback, and an integrated PWM control circuit. Its function is to provide precise angular position control within a defined operational arc, typically 180°.

## 2. Utilities
- High-torque, position-controlled actuation for robotic joints (e.g., pan/tilt systems, grippers).
- Control surface articulation in RC aeromodelling.
- Automated positioning systems for sensors and mechanical linkages.

## 3. Electronics and How It Works
Operation is based on a `PID or P-controller` feedback loop. A `50 Hz (20 ms period)` `PWM signal` serves as the setpoint. The pulse width `(duration of the high state)` is decoded by the internal controller. This setpoint is compared against the actual angular position, which is read from a potentiometer coupled to the `output shaft`. The resulting error signal drives an internal H-bridge, which powers the DC motor until the error is minimized (i.e., the potentiometer's output matches the `setpoint`).

## 4. How to Program It

Control is achieved by generating a 50 Hz PWM signal where the pulse width dictates the target angle.
- Neutral Position (90°): ~1500 µs pulse width.
- Min Position (0°): ~1000 µs pulse width.
- Max Position (180°): ~2000 µs pulse width.
  
### `C++ (Arduino Framework) Script`:
```ino
#include <ESP32Servo.h>
Servo servo;
void setup() { servo.attach(2); } // Pin 2
void loop() { servo.write(90); }   // Set angle to 90
```
### `Python (MicroPython) Script`
```py
from machine import Pin, PWM
pwm = PWM(Pin(2), freq=50) # Pin 2, 50Hz
# Duty cycle for 90 degrees (1.5ms pulse) is ~77 on a 0-1023 scale
pwm.duty(77) 
```
## 5. Connection to Arduino NANO ESP32
- VCC (Red): External 5V supply.
- GND (Brown/Black): Common ground with external supply and ESP32 GND.
- Signal (Orange/Yellow): Any PWM-capable GPIO on the ESP32 (e.g., D2).

## 6. Operating Voltage
- Standard Range: 4.8V - 6.0V DC.
- Current Draw: Quiescent: ~10-20mA. Stall Current: 500mA - 2.5A, depending on model and load.

## 7. Advantages and Disadvantages

### `Advantages:`
- High Power Density: Excellent torque-to-size ratio due to high-ratio gear trains.
- Simplified Control: Complex feedback loop is entirely self-contained.
- Holding Torque: Actively resists external forces to maintain a set position.

### `Disadvantages:`
- Limited Rotation: Standard models are constrained to a 180° arc.
- Positional Jitter: Can exhibit small oscillations around the setpoint under load.
- High Stall Current: Can damage power supplies or the servo itself if stalled.
- Gear Backlash: Mechanical play in the gearbox introduces positional inaccuracy.

# End Page
Seteki 2025 -  Components - Injora servo 11kg

