# HC-SR04
<img width="2000" height="1414" alt="Add a subheading (16 x 5 cm) (4096 x 2160 px) (A4 (Landscape))" src="https://github.com/user-attachments/assets/cf589c57-b04c-4b2c-af6d-eca1d5cad1f8" />


--

## 1. General Overview:
The `HC-SR04` is a widely-used, low-cost ultrasonic distance measuring sensor. It provides non-contact distance measurements from approximately `2cm to 400cm`. The module consists of an ultrasonic transmitter, a receiver, and a control circuit. It is a very popular component in the hobbyist and educational robotics communities due to its simplicity and effectiveness. By sending out a high-frequency sound pulse and measuring the time it takes for the echo to return, the sensor can calculate the distance to an object with reasonable accuracy.

---

## 2. Utilities:

The primary utility of the `HC-SR04` is distance measurement, which makes it suitable for a wide range of applications, including:
`Obstacle Avoidance:` A fundamental component in autonomous robots and drones to detect and navigate around obstacles.
`Distance Measurement`: Used in projects to measure the distance to a wall, object, or surface.
Level Sensing: Can be used to measure the level of liquids in a tank (e.g., water level) or solids in a container.
`Presence Detection`: Can be implemented in security systems or automated applications to detect if a person or object is within a certain area.
`Parking Assist Systems`: A common component in DIY car parking sensor projects.

---

## 3. Electronics and How It Works:
The `HC-SR04` operates on the principle of `sonar`. The process is as follows:
A short `10-microsecond high-level pulse` is sent to the Trig `(Trigger)` pin of the module.
In response, the module's transmitter sends out a burst of eight 40 kHz ultrasonic sound waves.
Simultaneously, the module's control circuit sets the Echo pin to a HIGH voltage level.
The sound waves travel through the air, hit an object, and reflect back towards the module.
When the receiver detects the reflected sound waves (the echo), the Echo pin is immediately set back to a LOW voltage level.

- The `microcontroller` measures the duration for which the Echo pin was HIGH. This duration is the round-trip travel time of the sound pulse.
  
The distance is then calculated using the formula: `Distance = (Duration of Echo Pulse × Speed of Sound) / 2.` The division by 2 is necessary because the pulse traveled to the object and back. The speed of sound is approximately 343 meters per second.

--

## 4. How to Program It:

Programming the sensor involves sending a trigger pulse and then precisely measuring the echo pulse duration.

### C++ (Arduino) Script:

```ino
// Define Trig and Echo pin connections
const int trigPin = 2;
const int echoPin = 3;

// Define variables for duration and distance
long duration;
int distance;

void setup() {
  Serial.begin(9600); // Initialize serial communication
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  // Clear the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Send a 10 microsecond pulse to trigger the sensor
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance (speed of sound is 0.0343 cm/µs)
  distance = duration * 0.0343 / 2;

  // Print the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  delay(500); // Wait for half a second before next reading
}

```

### Python (MicroPython/CircuitPython) Script:

```py

import machine
import time

# Define Trig and Echo pin connections
trig_pin = machine.Pin(2, machine.Pin.OUT)
echo_pin = machine.Pin(3, machine.Pin.IN)

def get_distance():
    # Send a 10 microsecond pulse to trigger the sensor
    trig_pin.value(0)
    time.sleep_us(2)
    trig_pin.value(1)
    time.sleep_us(10)
    trig_pin.value(0)

    # Wait for the echo pulse to start
    while echo_pin.value() == 0:
        pass
    start_time = time.ticks_us()

    # Wait for the echo pulse to end
    while echo_pin.value() == 1:
        pass
    end_time = time.ticks_us()

    # Calculate pulse duration and distance
    duration = time.ticks_diff(end_time, start_time)
    distance = (duration * 0.0343) / 2
    return distance

while True:
    dist = get_distance()
    print("Distance: {:.2f} cm".format(dist))
    time.sleep(0.5)

```

## 5. Connection to Arduino NANO ESP32:

- Connecting the `HC-SR04` to an `Arduino NANO ESP32` requires special attention to voltage levels.
  - `VCC pin of HC-SR04 connects to the 5V pin on the Arduino NANO ESP32.`
  - `GND pin of HC-SR04 connects to any GND pin on the Arduino.`
    
- Trig pin of HC-SR04 connects to any digital GPIO pin (e.g., D2). The 3.3V signal from the ESP32 is sufficient to trigger the 5V sensor.
  
- Echo pin of `HC-SR04` MUST be connected through a voltage divider before connecting to a digital GPIO pin (e.g., D3). This is because the HC-SR04 outputs a `5V signal`, which can damage the `3.3V-only GPIO` pins of the `ESP32`. A simple voltage divider can be made with a `1 kΩ and a 2 kΩ resistor`.
  
## 6. Operating Voltage:

The  `HC-SR04` operates at a standard voltage of 5V DC. While it may function at slightly lower voltages, its performance and range can be significantly degraded. For reliable operation, a 5V supply is required.

## 7. Advantages and Disadvantages:

### Advantages:
- `Low Cost`: Extremely affordable, making it accessible for any project.

- `Simple Interface:` Easy to use with only two signal pins (Trig and Echo).
  
- `Good Range:` Provides a decent measurement range (up to 4 meters) for its price.
Immune to Color/Light: Not affected by the color of the target object or ambient light conditions.

- `High Availability:` Widely available from numerous electronics suppliers.
  
### Disadvantages:

- `Narrow Beam:` The narrow detection cone can miss small or thin objects.
- `Surface Sensitivity:` Has difficulty detecting soft, irregularly shaped, or sound-absorbing materials (e.g., cloth, foam).
  
- `Angle Issues:` Angled surfaces can deflect the sound wave, preventing the echo from returning to the sensor.
  
- `Minimum Distance:` Has a minimum detection distance (around 2cm); objects closer than this cannot be measured accurately.
  
- `Interference:` Can be affected by other ultrasonic sensors operating nearby.
  
- `Environmental Factors:` Its accuracy can be slightly affected by changes in air temperature and humidity, which alter the speed of sound.

## How to buy it
[Sparkfun HC-SRO4](https://www.sparkfun.com/ultrasonic-distance-sensor-hc-sr04.html)

# End Page
Seteki 2025 - Components - HC-SR04

