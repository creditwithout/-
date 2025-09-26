# POLOLU MOTOR 800RPM 12V
![Add a subheading (16 x 5 cm) (4096 x 2160 px) (A4 (Landscape)) (2)](https://github.com/user-attachments/assets/014d874b-a253-47bc-898a-f88a3fc5cd62)

## 1. General Summary
This component is a high-power, motor integrated with a multi-stage metal gearbox and a quadrature Hall effect encoder. The assembly is designed for high-performance applications requiring precise closed-loop control over rotational velocity and position. 

## 2. Utilities
- Robotic Drivetrains: Primary actuator for mobile robots requiring accurate odometry and velocity profiling.
- PID Control Systems: Ideal for implementing precise speed (RPM) or position control loops.
- High-Torque Actuators: Used in robotic arms, lifts, and other mechanisms where both speed and positional feedback are critical.

## 3. Electronics
- Encoder: A two-channel Hall effect quadrature encoder is mounted to the extended motor shaft (pre-gearbox). It outputs two square waves (Channel A and Channel B) that are 90° out of phase.
- Resolution: The encoder provides a specific number of Counts Per Revolution (CPR) of the motor shaft. A typical value is 12 CPR.
- Output Resolution: The final resolution at the gearbox output shaft is (Motor Shaft CPR) × (Gear Ratio). For a 12 CPR encoder and a 10:1 gearbox, the output resolution is 12 × 10 = 120 counts per revolution.

## 4. How to Program It
Programming requires two distinct parts: driving the motor via an H-bridge and reading the encoder via hardware interrupts for accuracy.

### `C++ (Arduino Framework) Script:`
```ino
// Motor driver pins (e.g., TB6612FNG)
const int in1 = 2, in2 = 3, pwm = 4;
// Encoder pins with interrupts
const int encA = 5, encB = 6;
volatile long pulseCount = 0;

void IRAM_ATTR readEncoder() {
  // Increment/decrement count based on direction (phase of B relative to A)
  digitalRead(encB) > 0 ? pulseCount++ : pulseCount--;
}

void setup() {
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT); pinMode(pwm, OUTPUT);
  pinMode(encA, INPUT_PULLUP); pinMode(encB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encA), readEncoder, RISING);
}

void loop() {
  // Drive motor forward at ~50% speed
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  analogWrite(pwm, 128);
  // pulseCount variable is now tracking position
}
//This is not the actual CODE SETEKI USES, IT'S JUST A REFERENCE
```

```py
### `Python (MicroPython) Script`:
from machine import Pin, PWM
import time

# Motor driver pins
in1, in2 = Pin(2, Pin.OUT), Pin(3, Pin.OUT)
pwm = PWM(Pin(4), freq=1000)
# Encoder pins
encA = Pin(5, Pin.IN)
encB = Pin(6, Pin.IN)
pulse_count = 0

def encoder_handler(pin):
    global pulse_count
    if encB.value():
        pulse_count += 1
    else:
        pulse_count -= 1

# Attach interrupt
encA.irq(trigger=Pin.IRQ_RISING, handler=encoder_handler)

# Drive motor forward at 50% duty
in1.value(1)
in2.value(0)
pwm.duty_u16(32768) # 50% of 65535
#This is not the actual CODE SETEKI USES, IT'S JUST A REFERENCE
```
## 5. Connection to Arduino NANO ESP32:
- Motor Leads (+/-): Connect directly to the motor output terminals of an H-bridge driver (e.g., A01/A02 on a TB6612FNG).
- Encoder VCC (Red): Connect to 3.3V on the Nano ESP32.
- Encoder GND (Black): Connect to GND on the Nano ESP32. This ground must be common with the motor driver's logic ground.
- Encoder Channel A (Green): Connect to an interrupt-capable GPIO pin (e.g., D5).
- Encoder Channel B (Blue): Connect to any standard GPIO pin (e.g., D6).

## 6. Operating Voltage
- Motor Voltage (VM): 12V nominal (operable range typically 4.5V - 13.5V).
- Encoder Voltage (VCC): 3.3V to 5V.

## 7. Advantages and Disadvantages
### `Advantages`:
- Closed-Loop Control: Enables precise PID control of velocity and position.
- High Power Density: Delivers high torque and speed in a compact form factor.
- Odometry: Allows for accurate distance and turn tracking in mobile robots.

### `Disadvantages`:
- Increased Complexity: Requires a motor driver, more GPIO pins, and interrupt-based programming.
- Electrical Noise: Motor brush noise can potentially interfere with sensitive encoder signals, requiring careful wiring and possible filtering.
- Higher Cost: Significantly more expensive than a simple DC gearmotor.
