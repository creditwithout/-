# TB6612FNG
![Add a subheading (16 x 5 cm) (4096 x 2160 px) (A4 (Landscape))](https://github.com/user-attachments/assets/1465499f-0d6a-450c-9775-4fa62160bea2)

## 1. General Overview:

The `TB6612FNG` is a high-efficiency dual H-bridge driver `IC` designed for the bidirectional control of two `DC motors` or a single bipolar `stepper motor`. It is a significant improvement over older `BJT-based` drivers like the `L298N`, utilizing `MOSFETs` for its switching elements. This results in a lower voltage drop and higher power efficiency, making it an ideal choice for battery-powered `robotics` and mechatronic systems where power conservation is critical.

## 2. Utilities:

Bidirectional `DC Motor Control`: Independent control over the speed and direction of two small-to-medium `DC brushed motors`. 

### `Stepper Motor Control` :
Can drive a single bipolar stepper motor by controlling its `two coils`.

### `Robotic Actuation`: 
Widely used in mobile robot platforms, `robotic arms`, and other `electromechanical systems` requiring precise motor control.

## 3. Electronics and How It Works
The `TB6612FNG` contains two independent `H-bridge circuits`. An `H-bridge` consists of four `MOSFET switches arranged in an 'H'` configuration around the motor load. By selectively activating pairs of these switches, the driver can reverse the polarity of the voltage applied to the motor, thus controlling its direction of rotation.

### `Control is managed via a logic interface:`
- `IN1 / IN2`: These two digital inputs determine the motor's mode: forward, reverse, short brake `(braking with high torque)`, or stop `(coast)`.
- `PWM:` A Pulse-Width Modulation input controls the motor's speed. The duty cycle of the PWM signal effectively adjusts the average voltage supplied to the motor.
- `STBY:` A standby pin that, when pulled `LOW`, puts the driver into a low-power sleep mode, deactivating the H-bridges. It must be held HIGH for normal operation.
  
## 4. How to Program It:
Programming involves setting the direction with the IN pins and applying a PWM signal for speed.

### `C++ `(Arduino Framework)` Script`:

```ino
// Motor A connections
const int ain1 = 2;
const int ain2 = 3;
const int pwma = 4;
// Standby Pin
const int stby = 5;

void setup() {
  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  pinMode(pwma, OUTPUT);
  pinMode(stby, OUTPUT);
  
  // Bring driver out of standby
  digitalWrite(stby, HIGH);
}

void loop() {
  // Move forward at half speed
  digitalWrite(ain1, HIGH);
  digitalWrite(ain2, LOW);
  analogWrite(pwma, 128); // Speed from 0-255
  delay(2000);

  // Brake
  digitalWrite(ain1, HIGH);
  digitalWrite(ain2, HIGH);
  analogWrite(pwma, 0);
  delay(1000);
}
```
### `Python (MicroPython) Script`:

```py
from machine import Pin, PWM
import time

# Motor A connections
ain1 = Pin(2, Pin.OUT)
ain2 = Pin(3, Pin.OUT)
pwma = PWM(Pin(4))
# Standby Pin
stby = Pin(5, Pin.OUT)

# Set PWM frequency
pwma.freq(1000)

# Bring driver out of standby
stby.value(1)

def drive_motor(speed_percent, direction):
    # speed_percent is -100 to 100
    duty = int(abs(speed_percent / 100) * 65535)
    
    if direction == 1: # Forward
        ain1.value(1)
        ain2.value(0)
    else: # Reverse
        ain1.value(0)
        ain2.value(1)
    
    pwma.duty_u16(duty)

# Move forward at 75% speed
drive_motor(75, 1)
time.sleep(2)

# Stop motor (coast)
ain1.value(0)
ain2.value(0)
pwma.duty_u16(0)
```

