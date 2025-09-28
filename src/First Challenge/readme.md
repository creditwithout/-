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





