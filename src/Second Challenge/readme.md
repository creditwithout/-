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

## Block Avoidance System

### `Avoidance Parameters Configuration`
```ino
// Obstacle avoidance maneuver parameters
const float ANGULO_ESQUIVE = 40.0;            // Initial turn angle in degrees for standard avoidance
const float ANGULO_ESQUIVE_FUERTE = 45.0;     // Stronger turn angle in degrees for pronounced avoidance
const float TOLERANCIA_ANGULO_ESQUIVE = 3.0;  // Angular tolerance for completing avoidance turns
const float UMBRAL_PASILLO_LIBRE_CM = 35.0;   // Front distance threshold indicating clear passage
const int VEL_ESQUIVE_MIN = 120;              // Minimum speed during obstacle avoidance (PWM value)
const int VEL_ESQUIVE_MAX = 180;              // Maximum speed during obstacle avoidance (PWM value)
```

### `Blob Detection and Confirmation`
```ino
// Blob detection confirmation system
if (estadoActual != ESQUIVAR_BLOQUE && estadoActual != DETENIDO) {
    if (seccionBlob != 0) {
        blobDetectionCounter++;  // Increment confirmation counter
    } else {
        blobDetectionCounter = 0;  // Reset counter when no blob detected
    }

    if (blobDetectionCounter >= BLOB_CONFIRMATION_CYCLES) {  // 10 consecutive frames
        // Color and section gating logic
        bool esRojo = (seccionBlob >= 1 && seccionBlob <= 6);
        ColorBloque colorDetectado = esRojo ? COLOR_ROJO : COLOR_VERDE;
        bool permitirEsquive = true;
        
        // Block avoidance in curve sections
        if (bloquearEsquiveEnSeccion && enSeccionDeCurva) {
            permitirEsquive = false;
        }
        
        // Prevent re-avoidance of same color
        if (permitirEsquive && ultimoColorEsquivado != COLOR_NINGUNO && ultimoColorEsquivado == colorDetectado) {
            permitirEsquive = false;
        }
        
        if (permitirEsquive) {
            Serial.print("ESQUIVAR: Bloque confirmado. S: "); Serial.println(seccionBlob);
            estadoActual = ESQUIVAR_BLOQUE;  // Start avoidance maneuver
        } else {
            Serial.println("ESQUIVAR: Bloque ignorado por regla de color o seccion.");
        }
        blobDetectionCounter = 0;  // Reset for next detection
    }
}
```
### `Adaptative speed control`

```ino
// Adaptive speed based on front distance (soft anti-collision)
int velAdapt = VEL_ESQUIVE_MIN;  // Start with minimum speed
if (distanciaCentroFiltrada < 999.0f) {
    float d = distanciaCentroFiltrada;
    if (d < 15.0f) d = 15.0f;      // Clamp minimum distance
    if (d > 60.0f) d = 60.0f;      // Clamp maximum distance
    float t = (d - 15.0f) / (60.0f - 15.0f);  // Normalize distance (0-1)
    velAdapt = (int)(VEL_ESQUIVE_MIN + t * (VEL_ESQUIVE_MAX - VEL_ESQUIVE_MIN));  // Linear interpolation
}
moverAdelante(velAdapt);  // Apply adaptive speed
```

### `Avoidance Maneuver Planning`
```ino
if (subEstadoEsquivar == ES_NINGUNO) { // First time entering
    Serial.println(">>> ESQUIVAR: Evaluating maneuver...");
    yawInicialManiobra = currentYaw;  // Store initial yaw angle

    bool esRojo = (seccionBlob >= 1 && seccionBlob <= 6);
    colorEsquiveActual = esRojo ? COLOR_ROJO : COLOR_VERDE;
    bool necesitaEsquivar = true;
    
    // Default values for normal avoidance
    anguloEsquiveActual = ANGULO_ESQUIVE;

    if (esRojo) {
        direccionEsquive = IZQUIERDA; // Red -> Avoid to the Left
        Serial.print(">>> ESQUIVAR: RED block detected. ");
        if (roiBlob == 0) { // Left
            Serial.println("ROI 0 (Left), no avoidance needed.");
            necesitaEsquivar = false;
        } else if (roiBlob == 1) { // Center
            Serial.println("ROI 1 (Center), normal avoidance to LEFT.");
        } else { // roiBlob == 2 or fallback
            Serial.println("ROI 2 (Right), STRONG avoidance to LEFT.");
            anguloEsquiveActual = ANGULO_ESQUIVE_FUERTE;
        }
    } else { // Green
        direccionEsquive = DERECHA; // Green -> Avoid to the Right
        Serial.print(">>> ESQUIVAR: GREEN block detected. ");
        if (roiBlob == 2) { // Right
            Serial.println("ROI 2 (Right), no avoidance needed.");
            necesitaEsquivar = false;
        } else if (roiBlob == 1) { // Center
            Serial.println("ROI 1 (Center), normal avoidance to RIGHT.");
        } else { // roiBlob == 0 or fallback
            Serial.println("ROI 0 (Left), STRONG avoidance to RIGHT.");
            anguloEsquiveActual = ANGULO_ESQUIVE_FUERTE;
        }
    }

    if (necesitaEsquivar) {
        Serial.print(">>> Starting avoidance. Angle: "); Serial.println(anguloEsquiveActual);
        
        if (direccionEsquive == DERECHA) {
            setpointEsquive = yawInicialManiobra - anguloEsquiveActual;  // Turn right
        } else { // IZQUIERDA
            setpointEsquive = yawInicialManiobra + anguloEsquiveActual;  // Turn left
        }
        subEstadoEsquivar = ES_GIRO_INICIAL;  // Start with initial turn
    } else {
        Serial.println(">>> Avoidance maneuver not needed. Returning to NORMAL.");
        seccionBlob = 0; roiBlob = -1;  // Clear data to prevent re-activation
        estadoActual = NORMAL;
        subEstadoEsquivar = ES_NINGUNO;
    }
}
```

### `Three-Phase Avoidance Maneuver`
```ino
// Phase 1: Initial Turn
case ES_GIRO_INICIAL: {
    float error_esquive = calcularErrorAngular(setpointEsquive, currentYaw);
    if (fabs(error_esquive) < TOLERANCIA_ANGULO_ESQUIVE) {
        Serial.println(">>> ESQUIVAR: Initial turn complete. Advancing parallel.");
        subEstadoEsquivar = ES_AVANCE_PARALELO;  // Move to parallel advance
    }
    break;
}

// Phase 2: Parallel Advance
case ES_AVANCE_PARALELO: {
    bool pasilloLibre = (distanciaCentroFiltrada >= UMBRAL_PASILLO_LIBRE_CM);
    bool sinBlob = (seccionBlob == 0);
    if (pasilloLibre || sinBlob) {
        Serial.println(">>> ESQUIVAR: Clear passage detected. Returning to lane.");
        setpointEsquive = yawInicialManiobra;  // Point back to original angle
        subEstadoEsquivar = ES_GIRO_REGRESO;   // Move to return turn
    }
    break;
}

// Phase 3: Return Turn
case ES_GIRO_REGRESO: {
    float error_regreso = calcularErrorAngular(yawInicialManiobra, currentYaw);
    bool anguloAlcanzado = fabs(error_regreso) < TOLERANCIA_ANGULO_ESQUIVE;

    if (anguloAlcanzado) {
        Serial.println(">>> ESQUIVAR: Maneuver completed. Returning to NORMAL state.");
        setpoint = yawInicialManiobra;  // Restore original setpoint
        targetSetpoint = setpoint;
        integral = 0.0f;  // Reset PID
        seccionBlob = 0; roiBlob = -1;  // Clear blob data
        ultimoColorEsquivado = colorEsquiveActual;  // Remember avoided color
        colorEsquiveActual = COLOR_NINGUNO;
        estadoActual = NORMAL;
        subEstadoEsquivar = ES_NINGUNO;  // Reset sub-state
    }
    break;
}
```

- `The block avoidance` system implements intelligent obstacle detection and evasion using camera vision data combined with ultrasonic sensor feedback. The system operates through a three-phase maneuver: initial turn, parallel advance, and return turn, with color-coded obstacle recognition determining avoidance direction. Red blocks trigger left avoidance while green blocks trigger right avoidance, with ROI (Region of Interest) analysis determining avoidance intensity. The system employs adaptive speed control that scales velocity based on front obstacle distance, preventing collisions during avoidance maneuvers. 

- `Blob detection` requires 10 consecutive frame confirmations to prevent false triggers from camera noise or temporary obstructions. `Color-based` gating prevents re-avoidance of the same obstacle type, while section-based gating blocks avoidance during curve navigation phases. The avoidance state machine uses yaw angle control for precise steering, with separate tolerances for turn completion and parallel advance detection. Safety abort conditions monitor lateral and frontal distances, immediately canceling avoidance if safety `thresholds` are breached. The system integrates with the main navigation state machine, seamlessly returning to normal operation after successful obstacle clearance while maintaining navigation continuity and preventing repeated avoidance of the same obstacle.
# End Page
Seteki 2025


