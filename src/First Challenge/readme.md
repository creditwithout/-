# First Challenge Explanation
![Add a subheading (16 x 5 cm) (4096 x 2160 px) (9)](https://github.com/user-attachments/assets/6fa1c73f-00bd-4893-94fb-d86bc52d93b8)
### `We'll explain the most important tecnologies software in th code`

## PID

```ino
// Calculate angular error
error = calcularErrorAngular(setpoint_actual, currentYaw);  

// Apply deadband to prevent oscillation
if (fabs(error) <= deadband) { error = 0.0f; }  

// Store integral for anti-windup protection
float integralAntes = integral;  
if (!congelarPidYaw) {
    integral += error * dt_global;  // Update integral term
    integral = constrain(integral, -integralLimit, integralLimit);  // Apply integral windup protection
    derivative = (dt_global > 0.0001f) ? (error - prevError) / dt_global : 0.0f;  // Calculate derivative
    prevError = error;  // Store error for next derivative calculation
}

// Calculate PID output
pidOutput = (kp_actual * error) + (ki_actual * integral) + (kd_actual * derivative);  

// Anti-windup: if saturated under yaw control, revert integration of this cycle
bool controlYawActivo = !(estadoActual == PARKING && subEstadoParkingActual == SUB_PARKING_ALINEACION_LATERAL && intentandoAlineacionLateralActual) && !congelarPidYaw;
if (controlYawActivo && servoCommand != servoCommandSinSat) {
    integral = integralAntes;  // Revert integral to prevent windup
    pidOutput = (kp_actual * error) + (ki_actual * integral) + (kd_actual * derivative);  // Recalculate PID
    servoCommandSinSat = servoNeutral - pidOutput;  // Recalculate servo command
    servoCommand = constrain(servoCommandSinSat, servoMinAngle, servoMaxAngle);  // Apply limits again
}
```

This PID implementation demonstrates advanced control theory applied to autonomous robot navigation. The controller calculates angular error by comparing the desired setpoint with the current yaw reading from the IMU sensor. A deadband mechanism prevents servo oscillation when the error is within acceptable tolerance. The integral term accumulates error over time to eliminate steady-state error, while being constrained to prevent integral windup that could cause system instability. The derivative term calculates the rate of error change to provide damping and improve system response. The classic PID equation combines proportional, integral, and derivative terms with their respective gains (Kp, Ki, Kd) to generate the control output. The system implements sophisticated anti-windup protection by storing the previous integral value and reverting it if the servo command saturates, ensuring stable operation even when the actuator reaches its limits. This prevents the integral term from continuing to grow when the system cannot respond, which would cause overshoot and instability. The controller also includes a slew rate limiter that restricts how fast the servo can change position, preventing mechanical stress and ensuring smooth operation.

## State machine

### `State definition`

```ino
enum EstadoRobot { NORMAL, EN_MANIOBRA, ESTABILIZANDO, CORRIGIENDO, PARKING, DETENIDO };  // Robot state machine
EstadoRobot estadoActual = NORMAL;  // Current robot state
```

### `NORMAL State Example`

```ino

case NORMAL: {
     moverAdelante(velocidadNormal);  // Set normal navigation speed
     
     // Wall detection logic
     if (paredCercana) {  // Wall detected ahead
          if (direccionPrimerGiro == direccionDetectadaActual) {
               targetSetpoint = setpoint + (direccionPrimerGiro == IZQUIERDA ? -90.0f : 90.0f);  // Calculate turn target
               contadorCurvas++;                    // Increment turn counter
               estadoActual = EN_MANIOBRA;          // Change to maneuver state
          }
     }
     break;
}

```

### `EN_MANIOBRA State Example`

```ino

case EN_MANIOBRA: {
     moverAdelante(velocidadGiro);  // Set turning speed
     bool anguloAlcanzado = (fabs(error) < TURN_ANGLE_TOLERANCE);  // Check if turn angle reached
     bool tiempoManiobraTimeout = (tiempoTranscurrido >= TIMEOUT_MANIOBRA);  // Check timeout
     if (anguloAlcanzado || tiempoManiobraTimeout) {
          estabilizacionInicio = millis();  // Start stabilization timer
          estadoActual = ESTABILIZANDO;     // Change to stabilization state
     }
     break;
}

```

### `ESTABILIZANDO State Example`
```ino

case ESTABILIZANDO: {
     moverAdelante(velocidadEstabilizacion);  // Set stabilization speed
     bool tiempoEstabilizacionCumplido = (millis() - estabilizacionInicio >= tiempoEstabilizacion);  // Check stabilization time
     if (tiempoEstabilizacionCumplido) {
          correccionInicio = millis();          // Start correction timer
          estadoActual = CORRIGIENDO;          // Change to correction state
     }
     break;
}

```

- This finite state machine implements a sophisticated autonomous navigation system with six distinct operational states. The `NORMAL` state handles forward navigation with ultrasonic sensor monitoring to detect walls and openings, transitioning to `EN_MANIOBRA` when a turn is required. The `EN_MANIOBRA` state executes the actual turning maneuver using `PID control`, monitoring both angle completion and timeout conditions to ensure reliable operation. The ESTABILIZANDO state provides a brief stabilization period after each turn, allowing the robot to settle before proceeding to the `CORRIGIENDO` state for fine-tuning. The `CORRIGIENDO` state implements speed ramping and yaw stability checking, eventually transitioning to `PARKING` after completing the required number of turns.

- The `PARKING` state manages the final positioning sequence with lateral and frontal alignment algorithms. Each state transition is controlled by specific conditions such as sensor readings, timing constraints, and completion criteria, ensuring robust and predictable robot behavior throughout the navigation cycle.

## New Ping Library - HC-SRO4 Lectures

### `library initialization`
``` ino

#if USE_NEWPING
// Limiting maximum distance reduces echo wait time and speeds up each measurement
NewPing sonarIzq(TRIG_IZQUIERDO, ECHO_IZQUIERDO, MAX_US_DISTANCE_CM);
NewPing sonarCen(TRIG_CENTRO,    ECHO_CENTRO,    MAX_US_DISTANCE_CM);
NewPing sonarDer(TRIG_DERECHA,   ECHO_DERECHA,   MAX_US_DISTANCE_CM);

```

### `Asynchronous Echo Detection`

``` ino

enum SensorId { SEN_CEN = 0, SEN_IZQ = 1, SEN_DER = 2 };  // Sensor identification enum
volatile unsigned int echoUs[3] = {0, 0, 0};            // Echo timing results in microseconds
volatile bool echoListo[3] = {false, false, false};     // Echo completion flags
volatile bool pingEnCurso = false;                      // Current ping operation flag

void echoCheckCen() {
  if (sonarCen.check_timer()) {
    echoUs[SEN_CEN] = sonarCen.ping_result;  // Store center sensor echo timing
    echoListo[SEN_CEN] = true;               // Mark center echo as ready
    pingEnCurso = false;                     // Free up ping scheduler
  }
}

``` 

### `Fair Scheduling System`

```ino
void servicioUltrasonidos() {
  // Fair scheduling pattern: CEN -> IZQ -> CEN -> DER -> ...
  static uint8_t fase = 0; // 0=CEN,1=IZQ,2=CEN,3=DER
  
  // Try to find next available sensor in fair rotation pattern
  for (uint8_t intentos = 0; intentos < 4; intentos++) {
    uint8_t f = (fase + intentos) % 4;
    if ((f == 0 || f == 2) && puedeCen) {  // Center sensor slots (0 and 2)
      iniciarPing(SEN_CEN);
      ultimoPingCenMs = ahoraMs;
      fase = (f + 1) % 4;
      return;
    }
  }
}
```

### `Distance Measurement Processing`

```ino

if (echoListo[SEN_CEN]) {  // Check if center echo is ready
     noInterrupts();        // Disable interrupts for atomic operation
     unsigned int us = echoUs[SEN_CEN];  // Get echo timing
     echoListo[SEN_CEN] = false;         // Mark echo as processed
     interrupts();          // Re-enable interrupts
     float dist = (us == 0) ? 999.9 : (float)us / US_ROUNDTRIP_CM; // NewPing factor cm
     if (dist < DISTANCIA_MIN_VALIDA || dist > DISTANCIA_MAX_VALIDA) dist = 999.9;  // Validate distance
     lecturasCentro[indiceLecturaCentro] = dist;  // Add to filter buffer
}

```
The `NewPing library` provides `non-blocking` ultrasonic distance measurement capabilities through timer-based interrupt handling. Each sensor is initialized with trigger and echo pins plus a maximum distance limit that reduces measurement timeout periods. The system implements an asynchronous scheduler that manages three sensors using a fair rotation pattern `(CEN->IZQ->CEN->DER)`, ensuring no sensor monopolizes system resources while prioritizing center sensor readings for navigation accuracy. Echo detection occurs through callback functions that store timing results in volatile arrays and set completion flags, allowing the main loop to continue processing without blocking. 

- The `ping_timer()` method initiates measurements without waiting for results, while check_timer() verifies completion and retrieves timing data. Distance conversion uses the US_ROUNDTRIP_CM constant to transform microsecond timing into centimeter measurements. The system includes atomic operations with interrupt `disable/enable` sequences to prevent race conditions when accessing shared echo data. Validation filters eliminate readings outside acceptable ranges, and results are stored in circular buffers for median filtering. This architecture enables continuous robot operation while maintaining `high-frequency` sensor updates essential for real-time navigation and obstacle avoidance.


## BNO086

### `IMU Initialization`
```ino

BNO080 myIMU;        // IMU sensor object for orientation data

bool iniciarIMURobusto() {
  const uint8_t posiblesDirecciones[2] = {0x4B, 0x4A};  // Possible I2C addresses for BNO080
  const uint32_t clocks[4] = {100000, 50000, 20000, 400000};  // I2C clock speeds to try 
  for (uint8_t ci = 0; ci < 4; ci++) {
    Wire.setClock(clocks[ci]);
    for (uint8_t ai = 0; ai < 2; ai++) {
      uint8_t addr = posiblesDirecciones[ai];
      if (myIMU.begin(addr, Wire)) {
        Serial.println("OK");
        return true;
      }
    }
  }
}

```

### `Rotation Vector Configuration`
```ino

void setReports(void) {
  Serial.println("Enabling Rotation Vector...");
  myIMU.enableRotationVector(50);  // Enable rotation vector reports at 50Hz
  Serial.println(F("Rotation Vector enabled."));
}

```

### `Quaternion Processing`
```ino

if (myIMU.dataAvailable()) {
    ultimoTiempoDatosImu = millis();  // Update IMU watchdog timer
    float qI = myIMU.getQuatI();      // Get quaternion I component
    float qJ = myIMU.getQuatJ();      // Get quaternion J component
    float qK = myIMU.getQuatK();      // Get quaternion K component
    float qReal = myIMU.getQuatReal(); // Get quaternion real component
    
    float siny_cosp = 2.0 * (qReal * qK + qI * qJ);  // Calculate yaw sine term
    float cosy_cosp = 1.0 - 2.0 * (qJ * qJ + qK * qK); // Calculate yaw cosine term
    float yaw = atan2(siny_cosp, cosy_cosp);  // Calculate yaw angle in radians
    float currentYawDegrees = yaw * 180.0 / M_PI;  // Convert to degrees
}

```

### `Yaw Offset`
```ino
float yawOffset = 0.0f;  // Yaw offset for relative orientation
bool offsetSet = false;   // Flag indicating if yaw offset has been calibrated

if (!offsetSet) { 
    yawOffset = currentYawDegrees; 
    offsetSet = true; 
}  // Set initial offset
float relativeYaw = currentYawDegrees - yawOffset;  // Calculate relative yaw
if (relativeYaw > 180.0) { relativeYaw -= 360.0; }    // Normalize to [-180, 180]
else if (relativeYaw < -180.0) { relativeYaw += 360.0; }
currentYaw = -relativeYaw;  // Invert for robot coordinate system
```

### `Reset Detection`
```ino
if (myIMU.hasReset()) {
  Serial.print("Sensor reset. Reconfiguring... ");
  setReports();   // Reconfigure IMU reports after reset
  offsetSet = false;  // Reset offset flag for recalibration
}
```

The BNO080 is a 9-axis IMU sensor that combines accelerometer, gyroscope, and magnetometer data to provide accurate orientation measurements through quaternion mathematics. The system uses robust initialization that tries multiple I2C addresses `(0x4B, 0x4A)` and clock speeds to establish reliable communication, ensuring compatibility across different hardware configurations. The sensor operates in Rotation Vector mode at` 50Hz`, providing quaternion data that represents the device's orientation in 3D space without gimbal lock issues. Quaternion components `(qI, qJ, qK, qReal)` are processed using trigonometric transformations to extract yaw angle, converting from the complex quaternion representation to a simple angular measurement in degrees. The system implements automatic offset calibration on first reading, establishing a reference frame for relative navigation. Yaw angle normalization ensures values remain within the `[-180°, 180°] ` range, preventing discontinuities during continuous rotation. The coordinate system is inverted to match robot conventions where positive yaw represents right turns. Reset detection allows automatic reconfiguration if the sensor loses power or encounters communication errors, maintaining system reliability during extended operation.

## Quaternio to Euler angle conversion

### `Quaternion data extraction`
```ino

if (myIMU.dataAvailable()) {
    ultimoTiempoDatosImu = millis();  // Update IMU watchdog timer
    float qI = myIMU.getQuatI();      // Get quaternion I component
    float qJ = myIMU.getQuatJ();      // Get quaternion J component
    float qK = myIMU.getQuatK();      // Get quaternion K component
    float qReal = myIMU.getQuatReal(); // Get quaternion real component

```

### `Quaternion to Yaw Conversion (Euler Z-axis)`
```ino

// Quaternion to Euler conversion for Yaw (Z-axis rotation)
    float siny_cosp = 2.0 * (qReal * qK + qI * qJ);  // Calculate yaw sine term
    float cosy_cosp = 1.0 - 2.0 * (qJ * qJ + qK * qK); // Calculate yaw cosine term
    float yaw = atan2(siny_cosp, cosy_cosp);  // Calculate yaw angle in radians
    float currentYawDegrees = yaw * 180.0 / M_PI;  // Convert to degrees

```

### `Angle Normalization function`
```ino

float calcularErrorAngular(float target, float current) {
    float diff = target - current;  // Calculate raw difference
    while (diff <= -180.0) diff += 360.0;  // Normalize to [-180, 180] range
    while (diff > 180.0) diff -= 360.0;    // Normalize to [-180, 180] range
    return diff;  // Return normalized angular error
}

```

### `Target Setpoint Normalization`

```ino

targetSetpoint = setpoint + (direccionPrimerGiro == IZQUIERDA ? -90.0f : 90.0f);  // Calculate turn target

while (targetSetpoint <= -180.0f) targetSetpoint += 360.0f;  // Normalize angle
while (targetSetpoint > 180.0f) targetSetpoint -= 360.0f;

```


### `Relative yaw calculation`
```ino

if (!offsetSet) { 
    yawOffset = currentYawDegrees; 
    offsetSet = true; 
}  // Set initial offset
float relativeYaw = currentYawDegrees - yawOffset;  // Calculate relative yaw
if (relativeYaw > 180.0) { relativeYaw -= 360.0; }    // Normalize to [-180, 180]
else if (relativeYaw < -180.0) { relativeYaw += 360.0; }
currentYaw = -relativeYaw;  // Invert for robot coordinate system

```

- Quaternions represent 3D rotations using four components (w, x, y, z) that avoid gimbal lock and provide smooth interpolation, making them superior to Euler angles for robotics applications. The BNO080 provides quaternion data through `getQuatReal()`, `getQuatI()`, `getQuatJ()`, and `getQuatK()` methods, representing the scalar and vector components respectively. Euler angle extraction from quaternions requires trigonometric transformations: for yaw `(Z-axis rotation)`, the formula uses `2(wz + xy)` for the sine term and `1-2(y²+z²)` for the cosine term. The atan2() function converts these terms into a precise yaw angle in radians, which is then converted to degrees for human-readable navigation control. Angle normalization ensures all measurements remain within the `[-180°, 180°]` range, preventing discontinuities during continuous rotation and maintaining mathematical consistency.
