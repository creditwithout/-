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


## Anti-Collision System Implementation

###  `Safety Parameters Configuration`

```ino
// Safety system parameters for collision avoidance and emergency maneuvers
const float UMBRAL_LATERAL_SAFETY_CM = 18.0f;              // Lateral safety threshold for emergency corrections
const float YAW_SAFETY_BIAS_MAX = 12.0f;                   // Maximum yaw safety bias in degrees
const float YAW_SAFETY_BIAS_K = 0.6f;                      // Yaw bias gain per cm below threshold (deltaL - deltaR)
const float UMBRAL_FRONTAL_ABORT_ESQUIVE_CM = 15.0f;       // Front distance threshold to abort avoidance maneuver
const float UMBRAL_FRONTAL_SEGURIDAD_SECCION_CM = 20.0f;   // Front safety threshold in curve section
const float DISTANCIA_RETROCESO_SEGURIDAD_SECCION_CM = 15.0f;  // Safety reverse distance in curve section
const float LATERAL_OBJETIVO_PEGADO_CM = 75.0f;            // Target lateral distance for wall-following behavior
const float PEGADO_BIAS_DEG = 8.0f;                        // Steering bias in degrees for wall-following
```

### `Frontal Collision Prevention in NORMAL State`
```ino
case NORMAL: {
     if (distanciaCentroFiltrada < UMBRAL_DETENCION_FRONTAL) {
         detenerMotor();  // Emergency stop when front obstacle detected
     } else {
        moverAdelante(velocidadNormal);
        // Navigation logic continues...
     }
     break;
}
```

### `Lateral Safety Bias Calculation`
```ino

// Global lateral safety bias calculation
float deficitIzq = max(0.0f, UMBRAL_LATERAL_SAFETY_CM - distanciaIzquierdoFiltrada);
float deficitDer = max(0.0f, UMBRAL_LATERAL_SAFETY_CM - distanciaDerechoFiltrada);
float biasYaw = (deficitIzq - deficitDer) * YAW_SAFETY_BIAS_K; // positive: pushes right
biasYaw = constrain(biasYaw, -YAW_SAFETY_BIAS_MAX, YAW_SAFETY_BIAS_MAX);

// Apply yaw bias to servo command
if (biasYaw != 0.0f) {
    if (enReversa) {
        servoCommand = servoCommand + (int)biasYaw;
    } else {
        servoCommand = servoCommand - (int)biasYaw;
    }
}
```

### `Obstacle Avoidance Abort Conditions`
```ino
case ESQUIVAR_BLOQUE: {
    // Lateral safety abort conditions
    if (distanciaIzquierdoFiltrada < UMBRAL_LATERAL_SAFETY_CM || distanciaDerechoFiltrada < UMBRAL_LATERAL_SAFETY_CM) {
        Serial.println(">>> ESQUIVAR: Lateral too close, aborting and correcting yaw.");
        setpoint = yawInicialManiobra;
        targetSetpoint = setpoint;
        integral = 0.0f; derivative = 0.0f; prevError = 0.0f;
        ultimoColorEsquivado = (seccionBlob >= 1 && seccionBlob <= 6) ? COLOR_ROJO : COLOR_VERDE;
        seccionBlob = 0; roiBlob = -1;
        estadoActual = NORMAL;
        subEstadoEsquivar = ES_NINGUNO;
        break;
    }
    // Frontal safety abort conditions
    if (distanciaCentroFiltrada < UMBRAL_FRONTAL_ABORT_ESQUIVE_CM) {
        Serial.println(">>> ESQUIVAR: Front too close, aborting and correcting yaw.");
        setpoint = yawInicialManiobra;
        targetSetpoint = setpoint;
        integral = 0.0f; derivative = 0.0f; prevError = 0.0f;
        ultimoColorEsquivado = (seccionBlob >= 1 && seccionBlob <= 6) ? COLOR_ROJO : COLOR_VERDE;
        seccionBlob = 0; roiBlob = -1;
        estadoActual = NORMAL;
        subEstadoEsquivar = ES_NINGUNO;
        break;
    }
    // Continue avoidance logic...
}
```

### `Curve Section Emergency Recovery`
```ino
// Curve section supervisor: critical front -> recovery strategy
if (enSeccionDeCurva) {
    if (recSeccion == RC_NINGUNO && distanciaCentroFiltrada < UMBRAL_FRONTAL_SEGURIDAD_SECCION_CM) {
        Serial.println(">>> SECCION: Critical front in section. Starting recovery.");
        recSeccion = RC_DETENIENDO;
        pausarConteoSeccion = true; // don't count section distance during recovery
    }
    if (recSeccion != RC_NINGUNO) {
        switch (recSeccion) {
            case RC_DETENIENDO:
                detenerMotor();
                if (velocidadActualMotor == 0) {
                    resetEncoder();
                    recSeccion = RC_REVERSA;
                }
                break;
            case RC_REVERSA:
                moverAtras(velocidadGiro);
                if (fabs(leerDistanciaEncoder()) >= DISTANCIA_RETROCESO_SEGURIDAD_SECCION_CM) {
                    detenerMotor();
                    recSeccion = RC_CORRIGE_YAW;
                }
                break;
            // Additional recovery states...
        }
    }
}
```

### `Wall-Following Safety Enhancement`
```ino
// Temporary wall-following reinforcement during recovery
if (pegadoAParedActivo && ladoPegado != NINGUNA) {
    int pegadoBias = (ladoPegado == IZQUIERDA) ? -(int)PEGADO_BIAS_DEG : (int)PEGADO_BIAS_DEG;
    if (enReversa) {
        servoCommand = servoCommand + pegadoBias;
    } else {
        servoCommand = servoCommand - pegadoBias;
    }
}
```
The `anti-collision` system implements multi-layered safety mechanisms using ultrasonic sensor data to prevent robot collisions and ensure safe navigation. The system operates through three primary safety layers: `frontal collision prevention`, `lateral safety bias correction`, and emergency maneuver abort conditions. Frontal safety uses a single threshold `(10cm)` to immediately stop the robot when obstacles are detected directly ahead, preventing head-on collisions. Lateral safety calculates continuous yaw bias corrections based on the difference between left and right sensor readings, automatically steering away from walls when approaching safety thresholds. 

- The system employs differential safety calculations where left and right distance deficits are compared to generate proportional steering corrections, with maximum bias limits to prevent overcorrection. Emergency abort conditions monitor both lateral and frontal distances during obstacle avoidance maneuvers, immediately canceling avoidance operations and returning to normal navigation when safety thresholds are breached. The curve section recovery system provides specialized emergency handling during complex navigation phases, implementing controlled reverse maneuvers and yaw realignment when critical front obstacles are detected. `Wall-following` reinforcement adds temporary steering bias during recovery operations to maintain safe lateral positioning.

- All safety calculations use `atomic operations` and interrupt-safe data access to ensure real-time responsiveness and prevent data corruption during critical safety decisions.




