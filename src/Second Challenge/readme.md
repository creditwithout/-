# Second Challenge SRC Explanation
![Add a subheading (16 x 5 cm) (4096 x 2160 px) (10)](https://github.com/user-attachments/assets/f46b848f-8c14-4c5f-9891-7d8102ec5b3c)

### We'll explain the most important tecnologies software in the code, and the most magic of this round is made by the [camera](https://github.com/creditwithout/-/tree/main/src/Second%20Challenge/camera) .

## ENCODER

### `Configuration and Pin Definitions`

```ino
// Rotary encoder configuration for distance measurement
#define ENCODER_PIN_A 13        // Encoder channel A pin for quadrature detection
#define ENCODER_PIN_B A3        // Encoder channel B pin for quadrature detection (GPIO 39 on most ESP32)
const float PULSOS_POR_CM = 54.6;  // Calibrated pulses per centimeter for distance calculation
```

### `Global variables`
```ino
volatile long contadorPulsos = 0;  // Pulse counter for distance measurement
```

### `Interrupt Service Routine`
```ino
void IRAM_ATTR isr_encoder() {
  // Read current state of pin B to determine direction
  // If pulses are negative when advancing, ++ and -- should be inverted
  if (digitalRead(ENCODER_PIN_B)) {
    contadorPulsos--;  // Decrement for one direction
  } else {
    contadorPulsos++;  // Increment for opposite direction
  }
}
```

### `Encoder Initialization`
```ino
void setupEncoder() {
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);  // Configure pin A with pullup resistor
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);  // Configure pin B with pullup resistor
  
  // Configure interrupt to call 'isr_encoder'
  // every time pin A has a rising edge (RISING)
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), isr_encoder, RISING);
  
  Serial.println("Quadrature encoder initialized.");
}
```
### `Encoder Reset Function`

```ino
void resetEncoder() {
  // Disable interrupts to ensure atomic write operation
  noInterrupts();
  contadorPulsos = 0;  // Reset pulse counter to zero
  interrupts();        // Re-enable interrupts
}
```

### `Distance reading function`
```ino
float leerDistanciaEncoder() {
  long pulsos_copia;  // Local copy of pulse count
  
  // Disable interrupts, copy value, and re-enable interrupts
  // This ensures we read a complete value and not a corrupted one
  noInterrupts();
  pulsos_copia = contadorPulsos;  // Copy current pulse count
  interrupts();
  
  // Use simple calibration formula
  float distancia_cm = (float)pulsos_copia / PULSOS_POR_CM;  // Convert pulses to centimeters
  
  return distancia_cm;  // Return distance in centimeters
}
```

The rotary encoder system implements quadrature detection for precise distance measurement using `two-channel` phase-shifted signals. Channel A `triggers` interrupts on rising edges while Channel B provides direction information through phase relationship analysis. The interrupt service routine `(ISR)` uses the `IRAM_ATTR` attribute for optimal performance by placing critical timing code in internal `RAM`. Direction determination occurs by reading Channel B state during Channel A transitions, with pulse counting incrementing or decrementing based on phase relationship. The system employs atomic operations for thread-safe pulse counter access, using `noInterrupts()` and `interrupts()` to prevent data corruption during `read/write` operations. Calibration converts raw pulse counts to distance measurements using a calibrated constant `(54.6 pulses per centimeter)`, enabling accurate odometry for navigation algorithms. The encoder provides real-time distance feedback for maneuver execution, curve section management, and safety system operations. Reset functionality allows zeroing the distance counter at specific navigation waypoints, ensuring accurate relative positioning throughout the robot's journey.
