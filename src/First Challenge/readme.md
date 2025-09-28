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


