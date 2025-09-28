#include "SparkFun_BNO080_Arduino_Library.h"
#include <Wire.h>
#include <ESP32Servo.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#if __has_include(<NewPing.h>)
#include <NewPing.h>
#define USE_NEWPING 1
#else
#define USE_NEWPING 0
#endif
// ==================== CONFIGURABLE PARAMETERS SECTION ====================
// Hardware pin assignments for all robot components
const int servoPin = 8;        // Servo motor control pin for steering
const int pwma = 9;            // PWM pin for motor speed control (H-bridge)
const int ain2 = 10;           // Motor direction control pin A2 (H-bridge L298N)
const int ain1 = 11;           // Motor direction control pin A1 (H-bridge L298N)
const int stby = 12;           // Standby/enable pin for motor driver H-bridge
const int startButtonPin = A0; // Start button input pin with internal pullup
#define TRIG_IZQUIERDO 5    // Left ultrasonic sensor trigger pin
#define ECHO_IZQUIERDO 4     // Left ultrasonic sensor echo pin
#define TRIG_DERECHA 7       // Right ultrasonic sensor trigger pin
#define ECHO_DERECHA 6       // Right ultrasonic sensor echo pin
#define TRIG_CENTRO A2       // Center ultrasonic sensor trigger pin
#define ECHO_CENTRO A1       // Center ultrasonic sensor echo pin
#define CAM_RX_PIN A7        // Camera UART receive pin for serial communication
#define CAM_TX_PIN A6        // Camera UART transmit pin for serial communication

// IMU sensor configuration and I2C bus setup
const uint8_t I2C_SDA_PIN = A4;              // I2C data line pin for sensor communication
const uint8_t I2C_SCL_PIN = A5;              // I2C clock line pin for sensor communication
const uint8_t BNO08X_INT_PIN = 2;            // BNO080 IMU interrupt pin for data ready signal
const uint8_t BNO08X_RST_PIN = 3;            // BNO080 IMU reset pin for hardware reset capability
const uint8_t BNO08X_ADDR = 0x4B;            // I2C address of BNO080 IMU sensor
const uint16_t IMU_ROTATION_VECTOR_PERIOD_MS = 50;  // IMU data update rate in milliseconds (20Hz)

// Servo motor configuration and angle limits
const int servoNeutral = 90;   // Neutral position for straight ahead steering (90 degrees)
const int servoMinAngle = 60;  // Minimum servo angle limit for left steering (30 degrees from neutral)
const int servoMaxAngle = 120; // Maximum servo angle limit for right steering (30 degrees from neutral)

// PID controller gains for different navigation phases
float Kp = 1.5, Ki = 0.05, Kd = 0.8;           // Normal navigation PID gains (proportional, integral, derivative)
float Kp_turn = 2.0, Ki_turn = 0.02, Kd_turn = 0.7;  // Turning maneuver PID gains for sharper response
float Kp_stab = 1.2, Ki_stab = 0.03, Kd_stab = 1.0;  // Stabilization PID gains for smooth corrections
const unsigned long sampleInterval = 20;       // PID calculation interval in microseconds (50Hz)
const float integralLimit = 40.0;              // Integral windup protection limit to prevent overshoot
const float deadband = 0.5;                    // Deadband for forward motion to prevent oscillation
const float deadband_reversa = 1.5;            // Larger deadband for reverse motion due to different dynamics

// Motor speed ramping parameters for smooth acceleration and deceleration
const unsigned long MOTOR_RAMP_INTERVAL_MS = 20;  // Motor ramp update interval in milliseconds (50Hz)
const int MOTOR_ACCEL = 15;                       // Acceleration rate per update cycle (PWM units)
const int MOTOR_DECEL = 25;                       // Deceleration rate per update cycle (PWM units)

// Navigation and curve detection parameters
const int NUM_CURVAS_ANTES_DE_PARAR = 12;         // Maximum number of turns before parking sequence
const float UMBRAL_APERTURA_LATERAL = 80.0;       // Lateral opening threshold for turn detection
const float UMBRAL_DISTANCIA_FRONTAL_GIRO = 85.0; // Front distance threshold to initiate turn
const float DISTANCIA_SECCION_CURVA_CM = 110.0;   // Distance that defines the curve section after turn
const float UMBRAL_FRONTAL_PARA_INICIAR_GIRO_CM = 15.0;  // Front distance to start turning
const float DISTANCIA_REVERSA_C1_CM = 15.0;       // Reverse distance after turn in Lane 1
const float DISTANCIA_REVERSA_C2_CM = 20.0;       // Reverse distance after turn in Lane 2

// Lane detection thresholds based on lateral distance measurements
const float CARRIL_UMBRAL_1 = 40.0;  // Distance threshold to identify Lane 1 (closest to wall)
const float CARRIL_UMBRAL_2 = 55.0;  // Distance threshold to identify Lane 2 (middle lane)

// Ultrasonic sensor configuration and filtering parameters
const long TIMEOUT_ULTRASONICO = 20000;          // Ultrasonic measurement timeout in microseconds
const float DISTANCIA_MAX_VALIDA = 500.0;        // Maximum valid distance reading in centimeters
const float DISTANCIA_MIN_VALIDA = 1.0;          // Minimum valid distance reading in centimeters
#if USE_NEWPING
const unsigned int MAX_US_DISTANCE_CM = 200;     // Maximum measurable distance for NewPing library
#endif
const unsigned long MIN_INTERVALO_PING_MS = 15; // Minimum interval between ultrasonic pings in milliseconds

// Speed settings and timing parameters for different navigation phases
const int velocidadNormal = 180;                  // Normal forward navigation speed (PWM value)
const int velocidadGiro = 180;                   // Turning maneuver speed (PWM value)
const int VELOCIDAD_SALIDA_PARKING = 180;        // Speed for parking exit maneuvers (PWM value)
const unsigned long TIEMPO_REVERSA_INICIAL_MS = 800;   // Initial reverse time during parking exit
const unsigned long TIEMPO_AVANCE_SALIDA_MS = 1800;    // Forward movement time during parking exit
const float YAW_ALIGNED_TOLERANCE = 2.0;        // Yaw alignment tolerance for precise positioning
const float YAW_TOLERANCE_SECCION_CURVA = 15.0; // Yaw tolerance in curve section during obstacle avoidance
const float YAW_TOLERANCE_GIRO = 5.0;           // Yaw tolerance for completing 90-degree turns
const int VELOCIDAD_ALINEACION_YAW = 140;       // Speed for yaw alignment maneuvers (PWM value)
const unsigned long TIEMPO_REVERSA_FINAL_MS = 3000;    // Final reverse time during parking exit
const unsigned long TIMEOUT_YAW_ALIGN_MS = 2554;       // Timeout for yaw alignment operations

// Camera vision system and blob detection parameters
const int BLOB_CONFIRMATION_CYCLES = 10;       // Number of consecutive frames required to confirm blob detection
const float UMBRAL_DETENCION_FRONTAL = 10.0;   // Single threshold for stopping due to front proximity
const size_t CAM_BUFFER_SIZE = 64;             // Size of camera UART communication buffer

// Obstacle avoidance maneuver parameters
const float ANGULO_ESQUIVE = 40.0;            // Initial turn angle in degrees for standard avoidance
const float ANGULO_ESQUIVE_FUERTE = 45.0;     // Stronger turn angle in degrees for pronounced avoidance
const float TOLERANCIA_ANGULO_ESQUIVE = 3.0;  // Angular tolerance for completing avoidance turns
const float UMBRAL_PASILLO_LIBRE_CM = 35.0;   // Front distance threshold indicating clear passage
const int VEL_ESQUIVE_MIN = 120;              // Minimum speed during obstacle avoidance (PWM value)
const int VEL_ESQUIVE_MAX = 180;              // Maximum speed during obstacle avoidance (PWM value)

// Rotary encoder configuration for distance measurement
#define ENCODER_PIN_A 13        // Encoder channel A pin for quadrature detection
#define ENCODER_PIN_B A3        // Encoder channel B pin for quadrature detection (GPIO 39 on most ESP32)
const float PULSOS_POR_CM = 54.6;  // Calibrated pulses per centimeter for distance calculation

// Safety system parameters for collision avoidance and emergency maneuvers
const float UMBRAL_LATERAL_SAFETY_CM = 18.0f;              // Lateral safety threshold for emergency corrections
const float YAW_SAFETY_BIAS_MAX = 12.0f;                   // Maximum yaw safety bias in degrees
const float YAW_SAFETY_BIAS_K = 0.6f;                      // Yaw bias gain per cm below threshold (deltaL - deltaR)
const float UMBRAL_FRONTAL_ABORT_ESQUIVE_CM = 15.0f;       // Front distance threshold to abort avoidance maneuver
const float UMBRAL_FRONTAL_SEGURIDAD_SECCION_CM = 20.0f;   // Front safety threshold in curve section
const float DISTANCIA_RETROCESO_SEGURIDAD_SECCION_CM = 15.0f;  // Safety reverse distance in curve section
const float LATERAL_OBJETIVO_PEGADO_CM = 75.0f;            // Target lateral distance for wall-following behavior
const float PEGADO_BIAS_DEG = 8.0f;                        // Steering bias in degrees for wall-following
// ================= END OF CONFIGURABLE PARAMETERS =================
// ==================== GLOBAL PARAMETERS SECTION ====================
// Core sensor and control objects
BNO080 myIMU;                    // IMU sensor object for orientation and yaw data
Servo steeringServo;             // Servo motor object for steering control

// Yaw control and calibration variables
float yawOffset = 0.0f;          // Yaw offset for relative orientation measurement
bool offsetSet = false;          // Flag indicating if yaw offset has been calibrated
float setpoint = 0.0;            // Current yaw setpoint for PID control
float targetSetpoint = 0.0;      // Target yaw setpoint for maneuver planning

// PID controller state variables
float error = 0.0, prevError = 0.0;           // Current and previous yaw errors
float integral = 0.0, derivative = 0.0, pidOutput = 0.0;  // PID controller terms
unsigned long prevTimePID = 0;                // Previous PID calculation timestamp
float setpoint_actual_debug = 0.0;           // Debug variable for setpoint tracking
float dt_global = 0.02f;                     // Global delta time for PID calculations
float currentYaw = 0.0f;                     // Current relative yaw angle in degrees
// Main robot state machine enumeration
enum EstadoRobot { SALIR_PARKING, NORMAL, EN_MANIOBRA, CORRECCION_REVERSA, ESPERANDO_RESPUESTA_CAMARA, MANIOBRA_CARRIL_3, ESQUIVAR_BLOQUE, ESPERANDO_CONFIRMACION_PARED, DETENIDO };
EstadoRobot estadoActual = NORMAL;  // Current robot state
bool iniciandoReversa = false;      // Flag to manage safe transition to reverse motion

// Direction enumeration for turn management
enum DireccionGiro { NINGUNA, IZQUIERDA, DERECHA };  // Turn direction states

// Obstacle avoidance sub-state machine
enum SubEstadoEsquivar { ES_NINGUNO, ES_GIRO_INICIAL, ES_AVANCE_PARALELO, ES_GIRO_REGRESO };
SubEstadoEsquivar subEstadoEsquivar = ES_NINGUNO;  // Current avoidance sub-state
const char* subEstadoEsquivarNombres[] = {"NINGUNO", "GIRO_INI", "AVANCE_PAR", "GIRO_REG"};  // Sub-state names for debugging
// Maneuver execution sub-state machine for turn coordination
enum SubEstadoManiobra { M_INICIO, M_AVANCE_RECTO, M_GIRO_EN_CURVA };  // Maneuver sub-states
SubEstadoManiobra subEstadoManiobra = M_INICIO;  // Current maneuver sub-state
const char* subEstadoManiobraNombres[] = {"M_INICIO", "M_AVANCE", "M_GIRO"};  // Sub-state names for debugging
unsigned long tiempoInicioSubEstadoManiobra = 0;  // Maneuver sub-state start timestamp
float distanciaReversaObjetivo = 0.0f;           // Target reverse distance for CORRECCION_REVERSA state

// Obstacle avoidance maneuver variables
float yawInicialManiobra = 0.0f;    // Initial yaw angle when avoidance maneuver starts
float setpointEsquive = 0.0f;       // Target yaw setpoint during avoidance maneuver
DireccionGiro direccionEsquive = NINGUNA;  // Direction of current avoidance maneuver
float anguloEsquiveActual = 0.0f;   // Current avoidance turn angle in degrees

// State machine debugging and identification
const char* estadoNombres[] = {"SALIR_PARKING", "NORMAL", "EN_MANIOBRA", "CORRECCION_REVERSA", "ESPERANDO_RESP_CAM", "MANIOBRA_C3", "ESQUIVAR_BLOQUE", "ESP_CONF_PARED", "DETENIDO"};  // State names for debugging
// Parking exit sub-state machine for coordinated exit maneuvers
enum SubEstadoSalirParking { SP_INICIO, SP_REVERSA_GIRANDO, SP_AVANCE_SALIDA, SP_CORRIGIENDO_YAW, SP_REVERSA_FINAL, SP_ESPERANDO_CAMARA };
SubEstadoSalirParking subEstadoParking = SP_INICIO;  // Current parking exit sub-state
const char* subEstadoParkingNombres[] = {"INICIO", "REVERSA_INI", "AVANCE_SAL", "CORRIGE_YAW", "REVERSA_FIN", "ESP_CAM"};  // Sub-state names for debugging

// Lane identification and Lane 3 special maneuver handling
int carrilActual = 0;  // Current lane identification (0=unknown, 1=inner, 2=middle, 3=outer)
enum SubEstadoManiobraC3 { MC3_INICIO, MC3_AVANCE_RECTO, MC3_REVERSA_GIRANDO };  // Lane 3 maneuver sub-states
SubEstadoManiobraC3 subEstadoC3 = MC3_INICIO;  // Current Lane 3 maneuver sub-state
const char* subEstadoC3Nombres[] = {"INICIO_C3", "AVANCE_C3", "REVERSA_C3"};  // Sub-state names for debugging
// Navigation state tracking variables
unsigned int contadorCurvas = 0;           // Counter for completed turns in the circuit
DireccionGiro direccionPrimerGiro = NINGUNA;  // Direction of first turn (for consistency throughout circuit)

// Timing and state transition management
unsigned long tiempoInicioSubEstadoParking = 0;  // Parking sub-state start timestamp
unsigned long tiempoInicioSubEstadoC3 = 0;       // Lane 3 maneuver sub-state start timestamp
unsigned long tiempoUltimaManiobra = 0;          // Last maneuver completion timestamp

// Curve section and vision mask management
bool mascaraActivada = false;              // Flag indicating if camera vision mask is active
bool enSeccionDeCurva = false;             // Flag indicating if robot is in curve section
float encoderAlInicioDeCurva = 0.0f;      // Encoder position at start of curve section
// Ultrasonic sensor data and filtering
float distanciaIzquierdoFiltrada = 0.0;   // Filtered left ultrasonic distance reading
float distanciaCentroFiltrada = 0.0;      // Filtered center ultrasonic distance reading
float distanciaDerechoFiltrada = 0.0;     // Filtered right ultrasonic distance reading
float distanciaCentroRaw = 0.0;           // Raw center ultrasonic distance reading

// Camera communication and vision processing
int respuestaCamaraCheck = -1;            // Camera response for wall confirmation (8=block, 9=wall)
bool checkCompleto = false;               // Flag for CHECK response with counting
unsigned long ultimoTiempoDatosImu = 0;  // Last IMU data timestamp for watchdog

// Blob detection and tracking variables
int seccionBlob = 0;                      // Detected blob section (1-18 for different positions)
int roiBlob = -1;                         // Blob ROI: -1=None, 0=Left, 1=Center, 2=Right
int blobDetectionCounter = 0;             // Consecutive frame counter for blob confirmation
// Non-blocking UART buffer for camera communication
const size_t CAM_BUFFER_SIZE = 64;         // Camera communication buffer size
static char camBuffer[CAM_BUFFER_SIZE];    // Static buffer for camera UART data
static size_t camBufferLen = 0;            // Current buffer length

// Lane marking detection and Lane 1 restriction management
bool marcasCarrilDetectadasUltimoCheck = false;  // True if C,1 in the last CHECK (lane markings detected)
bool prohibirCarril1Seccion = false;             // Valid from section start until re-evaluation in next section
// Motor control and speed ramping variables
int velocidadActualMotor = 0;              // Current motor speed (PWM value)
int velocidadObjetivoMotor = 0;            // Target motor speed (PWM value)
enum DireccionMotor { MOTOR_DETENIDO, MOTOR_ADELANTE, MOTOR_ATRAS };  // Motor direction states
DireccionMotor direccionMotor = MOTOR_DETENIDO;  // Current motor direction

// Curve section initialization flag after CHECK command
bool iniciarSeccionTrasCheck = false;      // Flag to initiate curve section after CHECK response
// Safety system parameters and NewPing library configuration
const float UMBRAL_LATERAL_SAFETY_CM = 18.0f;              // Lateral safety threshold for emergency corrections
const float YAW_SAFETY_BIAS_MAX = 12.0f;                   // Maximum yaw safety bias in degrees
const float YAW_SAFETY_BIAS_K = 0.6f;                      // Yaw bias gain per cm below threshold (deltaL - deltaR)
const float UMBRAL_FRONTAL_ABORT_ESQUIVE_CM = 15.0f;       // Front distance threshold to abort avoidance maneuver
const float UMBRAL_FRONTAL_SEGURIDAD_SECCION_CM = 20.0f;   // Front safety threshold in curve section
const float DISTANCIA_RETROCESO_SEGURIDAD_SECCION_CM = 15.0f;  // Safety reverse distance in curve section
const float LATERAL_OBJETIVO_PEGADO_CM = 75.0f;            // Target lateral distance for wall-following behavior
const unsigned long MIN_INTERVALO_PING_MS = 22;            // Minimum interval between ultrasonic pings in milliseconds
const float PEGADO_BIAS_DEG = 8.0f;                        // Steering bias in degrees for wall-following

// NewPing library implementation for non-blocking ultrasonic readings (1rondaV2 scheme)
#if USE_NEWPING
// MAX_US_DISTANCE_CM is defined in the CONFIGURABLE PARAMETERS section
NewPing sonarIzq(TRIG_IZQUIERDO, ECHO_IZQUIERDO, MAX_US_DISTANCE_CM);  // Left ultrasonic sensor object
NewPing sonarCen(TRIG_CENTRO,    ECHO_CENTRO,    MAX_US_DISTANCE_CM);  // Center ultrasonic sensor object
NewPing sonarDer(TRIG_DERECHA,   ECHO_DERECHA,   MAX_US_DISTANCE_CM);  // Right ultrasonic sensor object

// Asynchronous ultrasonic sensor management variables
enum SensorId { SEN_CEN = 0, SEN_IZQ = 1, SEN_DER = 2 };  // Sensor identification enumeration
volatile unsigned int echoUs[3] = {0, 0, 0};              // Echo timing results in microseconds for each sensor
volatile bool echoListo[3] = {false, false, false};       // Echo completion flags for each sensor
volatile bool pingEnCurso = false;                        // Current ping operation status flag
static SensorId sensorEnCurso = SEN_CEN;                  // Currently active sensor being measured

// Echo detection callback functions for each ultrasonic sensor
void echoCheckCen() {
  if (sonarCen.check_timer()) {  // Check if center sensor measurement is complete
    echoUs[SEN_CEN] = sonarCen.ping_result;  // Store center sensor echo timing result
    echoListo[SEN_CEN] = true;               // Mark center echo as ready for processing
    pingEnCurso = false;                     // Free up ping scheduler for next measurement
  }
}
void echoCheckIzq() {
  if (sonarIzq.check_timer()) {  // Check if left sensor measurement is complete
    echoUs[SEN_IZQ] = sonarIzq.ping_result;  // Store left sensor echo timing result
    echoListo[SEN_IZQ] = true;               // Mark left echo as ready for processing
    pingEnCurso = false;                     // Free up ping scheduler for next measurement
  }
}
void echoCheckDer() {
  if (sonarDer.check_timer()) {  // Check if right sensor measurement is complete
    echoUs[SEN_DER] = sonarDer.ping_result;  // Store right sensor echo timing result
    echoListo[SEN_DER] = true;               // Mark right echo as ready for processing
    pingEnCurso = false;                     // Free up ping scheduler for next measurement
  }
}

// Function to initiate ultrasonic ping measurement on specified sensor
void iniciarPing(SensorId s) {
  if (pingEnCurso) return;  // Prevent overlapping ping operations
  switch (s) {
    case SEN_CEN: sonarCen.ping_timer(echoCheckCen); break;  // Start center sensor ping with callback
    case SEN_IZQ: sonarIzq.ping_timer(echoCheckIzq); break;  // Start left sensor ping with callback
    case SEN_DER: sonarDer.ping_timer(echoCheckDer); break;  // Start right sensor ping with callback
  }
  sensorEnCurso = s;     // Track which sensor is currently being measured
  pingEnCurso = true;    // Mark ping operation as in progress
}

// Fair scheduling service for ultrasonic sensors with priority to center sensor
void servicioUltrasonidos() {
  // Fair scheduling pattern: CEN -> IZQ -> CEN -> DER -> ... (center sensor gets priority)
  static uint8_t fase = 0; // 0=CEN,1=IZQ,2=CEN,3=DER (scheduling phase counter)
  unsigned long ahoraMs = millis();
  if (pingEnCurso) return;  // Skip if ping already in progress

  // Static variables to track last ping times for each sensor
  static unsigned long ultimoPingCenMs = 0;  // Last center sensor ping timestamp
  static unsigned long ultimoPingIzqMs = 0;  // Last left sensor ping timestamp
  static unsigned long ultimoPingDerMs = 0;  // Last right sensor ping timestamp
  
  // Check minimum interval constraints for each sensor to prevent interference
  bool puedeCen = (ahoraMs - ultimoPingCenMs) >= MIN_INTERVALO_PING_MS;  // Center sensor available
  bool puedeIzq = (ahoraMs - ultimoPingIzqMs) >= MIN_INTERVALO_PING_MS;  // Left sensor available
  bool puedeDer = (ahoraMs - ultimoPingDerMs) >= MIN_INTERVALO_PING_MS;  // Right sensor available

  // Try to find next available sensor in fair rotation pattern
  for (uint8_t intentos = 0; intentos < 4; intentos++) {
    uint8_t f = (fase + intentos) % 4;  // Calculate current phase in rotation
    if ((f == 0 || f == 2) && puedeCen) {  // Center sensor slots (0 and 2 for priority)
      iniciarPing(SEN_CEN);              // Start center sensor ping
      ultimoPingCenMs = ahoraMs;         // Update center sensor timestamp
      fase = (f + 1) % 4;                // Advance to next phase
      return;                            // Exit after successful ping initiation
    }
    if (f == 1 && puedeIzq) {  // Left sensor slot (1)
      iniciarPing(SEN_IZQ);    // Start left sensor ping
      ultimoPingIzqMs = ahoraMs;  // Update left sensor timestamp
      fase = (f + 1) % 4;        // Advance to next phase
      return;                    // Exit after successful ping initiation
    }
    if (f == 3 && puedeDer) {  // Right sensor slot (3)
      iniciarPing(SEN_DER);    // Start right sensor ping
      ultimoPingDerMs = ahoraMs;  // Update right sensor timestamp
      fase = (f + 1) % 4;        // Advance to next phase
      return;                    // Exit after successful ping initiation
    }
  }
}
#endif

// Obstacle color detection and avoidance blocking system
enum ColorBloque { COLOR_NINGUNO, COLOR_ROJO, COLOR_VERDE };  // Block color enumeration
ColorBloque ultimoColorEsquivado = COLOR_NINGUNO;           // Last color that was avoided (prevents re-avoidance)
ColorBloque colorEsquiveActual = COLOR_NINGUNO;             // Current avoidance maneuver color
bool bloquearEsquiveEnSeccion = false;                      // Flag to block avoidance in curve sections

// Curve section recovery system for emergency maneuvers
enum RecuperacionSeccionEstado { RC_NINGUNO, RC_DETENIENDO, RC_REVERSA, RC_CORRIGE_YAW, RC_PEGARSE, RC_FINALIZAR };  // Recovery state enumeration
RecuperacionSeccionEstado recSeccion = RC_NINGUNO;           // Current recovery state
unsigned long tiempoInicioRecuperacion = 0;                 // Recovery operation start timestamp
bool pausarConteoSeccion = false;                           // Flag to pause section distance counting during recovery
DireccionGiro ladoPegado = NINGUNA;                         // Side for wall-following behavior during recovery
bool pegadoAParedActivo = false;                            // Flag indicating active wall-following behavior
// ================= END OF GLOBAL PARAMETERS SECTION =================
void setReports(void) {
  Serial.println("Habilitando Rotation Vector...");
  myIMU.enableRotationVector(IMU_ROTATION_VECTOR_PERIOD_MS);
  Serial.println(F("Rotation Vector habilitado."));
}

// Robust IMU initialization with multiple I2C addresses and clock speeds
bool iniciarIMURobusto() {
    const uint8_t posiblesDirecciones[2] = {0x4B, 0x4A};  // Possible I2C addresses for BNO080
    const uint32_t clocks[4] = {100000, 50000, 20000, 400000};  // I2C clock speeds to attempt
    Serial.print("INT level before init: "); Serial.println(digitalRead(BNO08X_INT_PIN));
    
    // Try different I2C clock speeds and addresses for robust communication
    for (uint8_t ci = 0; ci < 4; ci++) {
        Wire.setClock(clocks[ci]);  // Set I2C clock speed
        for (uint8_t ai = 0; ai < 2; ai++) {
            uint8_t addr = posiblesDirecciones[ai];
            Serial.print("Trying IMU at 0x"); Serial.print(addr, HEX);
            Serial.print(" @I2C"); Serial.print(clocks[ci]/1000); Serial.print("k without INT/RST...");
            if (myIMU.begin(addr, Wire)) {  // Attempt IMU initialization
                Serial.println("OK");
                return true;  // Success: IMU found and initialized
            }
            Serial.println("fail");
            Serial.print("INT level after attempt without INT: "); Serial.println(digitalRead(BNO08X_INT_PIN));
        }
    }
    
    // If all attempts fail, scan I2C bus for debugging
    Serial.println("Scanning I2C bus...");
    for (uint8_t address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();
        if (error == 0) {
            Serial.print("I2C device found at 0x");
            Serial.println(address, HEX);
        }
    }
    return false;  // IMU initialization failed
}
// Measure distance using ultrasonic sensor with validation and error handling
float medirDistancia(int trigPin, int echoPin) {
#if USE_NEWPING
    unsigned int cm = 0;
    // Route measurement to appropriate sensor based on pin configuration
    if (trigPin == TRIG_IZQUIERDO && echoPin == ECHO_IZQUIERDO) {
        cm = sonarIzq.ping_cm();  // Get left sensor distance measurement
    } else if (trigPin == TRIG_CENTRO && echoPin == ECHO_CENTRO) {
        cm = sonarCen.ping_cm();  // Get center sensor distance measurement
    } else if (trigPin == TRIG_DERECHA && echoPin == ECHO_DERECHA) {
        cm = sonarDer.ping_cm();  // Get right sensor distance measurement
    } else {
        cm = 0;  // Invalid pin combination
    }
    if (cm == 0) return 999.9;  // Return invalid distance if no echo received
    float dist = (float)cm;
    if (dist < DISTANCIA_MIN_VALIDA || dist > DISTANCIA_MAX_VALIDA) return 999.9;  // Validate distance range
    return dist;
#else
    // Fallback implementation using direct pulseIn for systems without NewPing library
    digitalWrite(trigPin, LOW); delayMicroseconds(2);     // Clear trigger pin
    digitalWrite(trigPin, HIGH); delayMicroseconds(10);   // Send 10us trigger pulse
    digitalWrite(trigPin, LOW);                           // Clear trigger pin
    long duration = pulseIn(echoPin, HIGH, TIMEOUT_ULTRASONICO);  // Measure echo duration
    if (duration == 0) return 999.9;  // Return invalid if timeout
    float dist = (duration * 0.0343) / 2.0;  // Convert to distance (cm) using speed of sound
    if (dist < DISTANCIA_MIN_VALIDA || dist > DISTANCIA_MAX_VALIDA) return 999.9;  // Validate range
    return dist;
#endif
}
void moverAdelante(int velocidad) {
    direccionMotor = MOTOR_ADELANTE;
    velocidadObjetivoMotor = constrain(velocidad, 0, 255);
}
void detenerMotor() {
    direccionMotor = MOTOR_DETENIDO;
    velocidadObjetivoMotor = 0;
}
float calcularErrorAngular(float target, float current) {
    float diff = target - current;
    while (diff <= -180.0) diff += 360.0;
    while (diff > 180.0) diff -= 360.0;
    return diff;
}
void moverAtras(int velocidad) {
    direccionMotor = MOTOR_ATRAS;
    velocidadObjetivoMotor = constrain(velocidad, 0, 255);
}
void procesarLineaCamara(const char* linea) {
  size_t n = strlen(linea);
  if (n < 3) return;
  char tipo = linea[0];
  const char* coma = strchr(linea, ',');
  const char* datos = (coma && (coma + 1) < (linea + n)) ? (coma + 1) : NULL;
  if (!datos) return;

  switch (tipo) {
    case 'P': { // P,8 o P,9
      if (estadoActual == ESPERANDO_CONFIRMACION_PARED && datos) {
        respuestaCamaraCheck = atoi(datos);
      }
      break;
    }
    case 'C': { // C,x
      bool esperando = (estadoActual == ESPERANDO_RESPUESTA_CAMARA) ||
                       (estadoActual == SALIR_PARKING && subEstadoParking == SP_ESPERANDO_CAMARA);
      // Parsear x (0/1) para saber si hay marcas de carril detectadas por cámara
      int valor = atoi(datos);
      marcasCarrilDetectadasUltimoCheck = (valor != 0);
      if (esperando) {
        checkCompleto = true;
      }
      break;
    }
    case 'B': { // B,sec,roi
      // Buscar segunda coma
      const char* coma2 = strchr(datos, ',');
      if (coma2) {
        char tmpSec[8];
        char tmpRoi[8];
        size_t lenSec = (size_t)(coma2 - datos);
        size_t lenRoi = (size_t)((linea + n) - (coma2 + 1));
        if (lenSec > 0 && lenSec < sizeof(tmpSec) && lenRoi > 0 && lenRoi < sizeof(tmpRoi)) {
          memcpy(tmpSec, datos, lenSec); tmpSec[lenSec] = '\0';
          memcpy(tmpRoi, coma2 + 1, lenRoi); tmpRoi[lenRoi] = '\0';
          seccionBlob = atoi(tmpSec);
          roiBlob = atoi(tmpRoi);
        } else {
          seccionBlob = 0; roiBlob = -1;
        }
      } else {
        seccionBlob = 0; roiBlob = -1;
      }
      break;
    }
    default:
      break;
  }
}

void procesarSerialCamara() {
  while (Serial1.available() > 0) {
    char c = (char)Serial1.read();
    if (c == '\r') continue; // ignorar CR
    if (c == '\n') {
      camBuffer[camBufferLen] = '\0';
      if (camBufferLen > 0) {
        procesarLineaCamara(camBuffer);
      }
      camBufferLen = 0;
    } else {
      if (camBufferLen < CAM_BUFFER_SIZE - 1) {
        camBuffer[camBufferLen++] = c;
      } else {
        // overflow: resetear línea
        camBufferLen = 0;
      }
    }
  }
}
void actualizarMotorConRampa() {
    static unsigned long ultimoTiempoRampa = 0;
    static DireccionMotor direccionRealMotor = MOTOR_DETENIDO;
    unsigned long ahora = millis();
    if (ahora - ultimoTiempoRampa < MOTOR_RAMP_INTERVAL_MS) {
        return;
    }
    ultimoTiempoRampa = ahora;

    if (direccionMotor != direccionRealMotor && velocidadActualMotor > 0) {
        velocidadActualMotor = max(0, velocidadActualMotor - MOTOR_DECEL);
    } else {
        direccionRealMotor = direccionMotor;
        if (velocidadActualMotor < velocidadObjetivoMotor) {
            velocidadActualMotor = min(velocidadObjetivoMotor, velocidadActualMotor + MOTOR_ACCEL);
        } else if (velocidadActualMotor > velocidadObjetivoMotor) {
            velocidadActualMotor = max(velocidadObjetivoMotor, velocidadActualMotor - MOTOR_DECEL);
        }
    }

    switch (direccionRealMotor) {
        case MOTOR_ADELANTE:
            digitalWrite(ain2, LOW); digitalWrite(ain1, HIGH);
            analogWrite(pwma, velocidadActualMotor);
            break;
        case MOTOR_ATRAS:
            digitalWrite(ain1, LOW); digitalWrite(ain2, HIGH);
            analogWrite(pwma, velocidadActualMotor);
            break;
        case MOTOR_DETENIDO:
            digitalWrite(ain1, LOW); digitalWrite(ain2, LOW);
            analogWrite(pwma, 0);
            velocidadActualMotor = 0;
            break;
    }
}
// (Eliminado sanitizeDist: no utilizado)

void actualizarFiltrosSensores() {
     static bool leerIzquierdaEstaVez = true;

     unsigned long ahoraMs = millis();
#if USE_NEWPING
     if (echoListo[SEN_CEN]) {
         noInterrupts();
         unsigned int us = echoUs[SEN_CEN];
         echoListo[SEN_CEN] = false;
         interrupts();
         float dist = (us == 0) ? 999.9 : (float)us / US_ROUNDTRIP_CM;
         if (dist < DISTANCIA_MIN_VALIDA || dist > DISTANCIA_MAX_VALIDA) dist = 999.9;
         distanciaCentroRaw = dist;
         distanciaCentroFiltrada = dist; // FILTER_SIZE=1
     }
#else
     static unsigned long ultimoPingCenMs = 0;
     if (ahoraMs - ultimoPingCenMs >= MIN_INTERVALO_PING_MS) {
         float lecturaRawCen = medirDistancia(TRIG_CENTRO, ECHO_CENTRO);
         distanciaCentroRaw = lecturaRawCen;
         distanciaCentroFiltrada = lecturaRawCen;
         ultimoPingCenMs = ahoraMs;
     }
#endif

     if (leerIzquierdaEstaVez) {
#if USE_NEWPING
         if (echoListo[SEN_IZQ]) {
             noInterrupts();
             unsigned int us = echoUs[SEN_IZQ];
             echoListo[SEN_IZQ] = false;
             interrupts();
             float dist = (us == 0) ? 999.9 : (float)us / US_ROUNDTRIP_CM;
             if (dist < DISTANCIA_MIN_VALIDA || dist > DISTANCIA_MAX_VALIDA) dist = 999.9;
             distanciaIzquierdoFiltrada = dist; // FILTER_SIZE=1
         }
#else
         static unsigned long ultimoPingIzqMs = 0;
         if (ahoraMs - ultimoPingIzqMs >= MIN_INTERVALO_PING_MS) {
             float lecturaRawIzq = medirDistancia(TRIG_IZQUIERDO, ECHO_IZQUIERDO);
             distanciaIzquierdoFiltrada = lecturaRawIzq;
             ultimoPingIzqMs = ahoraMs;
         }
#endif
     } else {
#if USE_NEWPING
         if (echoListo[SEN_DER]) {
             noInterrupts();
             unsigned int us = echoUs[SEN_DER];
             echoListo[SEN_DER] = false;
             interrupts();
             float dist = (us == 0) ? 999.9 : (float)us / US_ROUNDTRIP_CM;
             if (dist < DISTANCIA_MIN_VALIDA || dist > DISTANCIA_MAX_VALIDA) dist = 999.9;
             distanciaDerechoFiltrada = dist; // FILTER_SIZE=1
         }
#else
         static unsigned long ultimoPingDerMs = 0;
         if (ahoraMs - ultimoPingDerMs >= MIN_INTERVALO_PING_MS) {
             float lecturaRawDer = medirDistancia(TRIG_DERECHA, ECHO_DERECHA);
             distanciaDerechoFiltrada = lecturaRawDer;
             ultimoPingDerMs = ahoraMs;
         }
#endif
     }
     leerIzquierdaEstaVez = !leerIzquierdaEstaVez;
}
void setup() {
    Serial.begin(115200);
    pinMode(BNO08X_INT_PIN, INPUT_PULLUP);
    pinMode(BNO08X_RST_PIN, OUTPUT);
    digitalWrite(BNO08X_RST_PIN, HIGH);
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);
    Serial1.begin(115200, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);
    Serial.println("2rondaV2.0");
    steeringServo.attach(servoPin);
    steeringServo.write(servoNeutral);
    if (!iniciarIMURobusto()) {
        Serial.println(F("BNO08x no detectado"));
        while (1);
    }
    Serial.println(F("BNO08x encontrado."));
    setReports();
    delay(100);
    // Rotation Vector con offsetSet: sin calibración bloqueante
    setupEncoder();
    ultimoTiempoDatosImu = millis();
    // Configurar pines de US (válido también para fallback pulseIn)
    pinMode(TRIG_IZQUIERDO, OUTPUT); pinMode(ECHO_IZQUIERDO, INPUT);
    pinMode(TRIG_CENTRO, OUTPUT); pinMode(ECHO_CENTRO, INPUT);
    pinMode(TRIG_DERECHA, OUTPUT); pinMode(ECHO_DERECHA, INPUT);

    Serial.println("Llenando buffers de filtro iniciales...");
    distanciaIzquierdoFiltrada = medirDistancia(TRIG_IZQUIERDO, ECHO_IZQUIERDO);
    distanciaCentroRaw = medirDistancia(TRIG_CENTRO, ECHO_CENTRO);
    distanciaCentroFiltrada = distanciaCentroRaw;
    distanciaDerechoFiltrada = medirDistancia(TRIG_DERECHA, ECHO_DERECHA);
    pinMode(pwma, OUTPUT); pinMode(ain2, OUTPUT); pinMode(ain1, OUTPUT); pinMode(stby, OUTPUT);
    digitalWrite(stby, HIGH);
    detenerMotor();
    currentYaw = 0.0f;
    setpoint = 0.0f;
    targetSetpoint = 0.0;
    prevTimePID = micros();
    estadoActual = SALIR_PARKING;
    subEstadoParking = SP_INICIO;
    pinMode(startButtonPin, INPUT_PULLUP);
    Serial.println("Calibracion completa. Presione el boton para iniciar...");
    while (digitalRead(startButtonPin) == HIGH) {
        delay(50);
    }
    Serial.println("Boton presionado. Iniciando movimiento!");
}
void actualizarSeccionDeCurva() {
  if (enSeccionDeCurva) {
    if (pausarConteoSeccion) {
      return;
    }
    float distanciaRecorrida = fabs(leerDistanciaEncoder() - encoderAlInicioDeCurva);
    if (distanciaRecorrida >= DISTANCIA_SECCION_CURVA_CM) {
      enSeccionDeCurva = false;
      Serial.println(">>> SECCION: Finalizando seccion de curva.");
      if (mascaraActivada) {
          Serial1.write('N');
          Serial.println(">>> CMD: Mascara NORMAL (vision completa)");
          mascaraActivada = false;
      }
      // Resetear restriccion de carril 1 al finalizar la seccion
      prohibirCarril1Seccion = false;
      // Reactivar esquives para nuevas secciones
      bloquearEsquiveEnSeccion = false;
    }
  }
}
void manejarReinicioIMU() {
    Serial.println("¡Alerta! El giroscopio se ha reiniciado. Reconfigurando reportes...");
    detenerMotor();
    steeringServo.write(servoNeutral);
    setReports();
    // Reiniciar offset como en 1rondaV2: se fijará en la próxima lectura
    offsetSet = false;
    integral = 0.0f;
    derivative = 0.0f;
    prevError = 0.0f;
    prevTimePID = micros();
}
void loop() {
    if (estadoActual == DETENIDO) {
        detenerMotor();
        actualizarMotorConRampa();
        digitalWrite(stby, LOW);
        return;
    }
    actualizarMotorConRampa();
    actualizarSeccionDeCurva();
    if (myIMU.hasReset()) {
      manejarReinicioIMU();
      return;
    }
    if (myIMU.dataAvailable()) {
        ultimoTiempoDatosImu = millis();
        float qI = myIMU.getQuatI();
        float qJ = myIMU.getQuatJ();
        float qK = myIMU.getQuatK();
        float qReal = myIMU.getQuatReal();
        float siny_cosp = 2.0 * (qReal * qK + qI * qJ);
        float cosy_cosp = 1.0 - 2.0 * (qJ * qJ + qK * qK);
        float yaw = atan2(siny_cosp, cosy_cosp);
        float currentYawDegrees = yaw * 180.0 / M_PI;
        if (!offsetSet) { yawOffset = currentYawDegrees; offsetSet = true; }
        float relativeYaw = currentYawDegrees - yawOffset;
        if (relativeYaw > 180.0f) relativeYaw -= 360.0f;
        else if (relativeYaw < -180.0f) relativeYaw += 360.0f;
        currentYaw = -relativeYaw;
    }
    procesarSerialCamara();
#if USE_NEWPING
    servicioUltrasonidos();
#endif
    actualizarFiltrosSensores();

    if (estadoActual != ESQUIVAR_BLOQUE && estadoActual != DETENIDO) {
        if (seccionBlob != 0) {
            blobDetectionCounter++;
        } else {
            blobDetectionCounter = 0;
        }

        if (blobDetectionCounter >= BLOB_CONFIRMATION_CYCLES) {
            if(estadoActual == EN_MANIOBRA) {
                integral = 0.0f; derivative = 0.0f;
            }
            // --- NUEVO: Gating por color y por sección ---
            bool esRojo = (seccionBlob >= 1 && seccionBlob <= 6);
            ColorBloque colorDetectado = esRojo ? COLOR_ROJO : COLOR_VERDE;
            bool permitirEsquive = true;
            if (bloquearEsquiveEnSeccion && enSeccionDeCurva) {
                permitirEsquive = false;
            }
            if (permitirEsquive && ultimoColorEsquivado != COLOR_NINGUNO && ultimoColorEsquivado == colorDetectado) {
                permitirEsquive = false;
            }
            if (permitirEsquive) {
                Serial.print("ESQUIVAR: Bloque confirmado. S: "); Serial.println(seccionBlob);
                estadoActual = ESQUIVAR_BLOQUE;
            } else {
                Serial.println("ESQUIVAR: Bloque ignorado por regla de color o seccion.");
            }
            blobDetectionCounter = 0; // Resetear para la proxima
        }
    }

    actualizarControl();
    manejarEstados();
    static unsigned long lastPrintTime = 0;
    if (millis() - lastPrintTime > 300) {
        Serial.print("estado:"); Serial.print(estadoNombres[estadoActual]);
        if (estadoActual == SALIR_PARKING) {
            Serial.print("/"); Serial.print(subEstadoParkingNombres[subEstadoParking]);
        }
        if (estadoActual == EN_MANIOBRA) {
            Serial.print("/"); Serial.print(subEstadoManiobraNombres[subEstadoManiobra]);
        }
        if (estadoActual == MANIOBRA_CARRIL_3) {
            Serial.print("/"); Serial.print(subEstadoC3Nombres[subEstadoC3]);
        }
        if (estadoActual == ESQUIVAR_BLOQUE) {
            Serial.print("/"); Serial.print(subEstadoEsquivarNombres[subEstadoEsquivar]);
        }
        Serial.print("|Carril:"); Serial.print(carrilActual);
        Serial.print("|Yaw:"); Serial.print(currentYaw, 1);
        Serial.print("|SetP:"); Serial.print(setpoint_actual_debug, 1);
        Serial.print("|ErrY:"); Serial.print(error, 1);
        Serial.print("|DI:"); Serial.print(distanciaIzquierdoFiltrada, 0);
        Serial.print("|DCent:"); Serial.print(distanciaCentroRaw, 0);
        Serial.print("|DD:"); Serial.print(distanciaDerechoFiltrada, 0);
        Serial.print("|Sect:"); Serial.print(seccionBlob);
        Serial.print("|Roi:"); Serial.print(roiBlob);
        Serial.print("|curv:"); Serial.print(contadorCurvas);
        Serial.print("|SC:"); Serial.print(enSeccionDeCurva);
        Serial.print("|Srv:"); Serial.print(steeringServo.read());
        Serial.println();
        lastPrintTime = millis();
    }
}

void actualizarControl() {
    unsigned long currentTime = micros();
    unsigned long elapsedTime = currentTime - prevTimePID;
    if (elapsedTime <= 0) elapsedTime = sampleInterval * 1000UL;
    dt_global = elapsedTime / 1000000.0f;
    prevTimePID = currentTime;
    if (estadoActual == DETENIDO || estadoActual == ESPERANDO_RESPUESTA_CAMARA || estadoActual == ESPERANDO_CONFIRMACION_PARED) {
        integral = 0.0;
        return;
    }

    float kp_actual, ki_actual, kd_actual;
    float setpoint_actual_yaw;
    switch(estadoActual) {
        case SALIR_PARKING:
            kp_actual = Kp; ki_actual = Ki; kd_actual = Kd;
            setpoint_actual_yaw = 0.0f;
            break;
        case EN_MANIOBRA:
            if (subEstadoManiobra == M_AVANCE_RECTO) {
                kp_actual = Kp; ki_actual = Ki; kd_actual = Kd;
                setpoint_actual_yaw = setpoint;
            } else { // M_GIRO_EN_CURVA o M_INICIO
                kp_actual = Kp_turn; ki_actual = Ki_turn; kd_actual = Kd_turn;
                setpoint_actual_yaw = targetSetpoint;
            }
            break;
        case MANIOBRA_CARRIL_3:
            if (subEstadoC3 == MC3_AVANCE_RECTO) {
                kp_actual = Kp; ki_actual = Ki; kd_actual = Kd;
                setpoint_actual_yaw = setpoint;
            } else {
                kp_actual = Kp_turn; ki_actual = Ki_turn; kd_actual = Kd_turn;
                setpoint_actual_yaw = targetSetpoint;
            }
            break;
        case CORRECCION_REVERSA: {
            kp_actual = Kp_stab;
            ki_actual = Ki_stab;
            kd_actual = Kd_stab;
            setpoint_actual_yaw = setpoint;
            break;
        }
        case NORMAL: {
            kp_actual = Kp;
            ki_actual = Ki;
            kd_actual = Kd;
            setpoint_actual_yaw = setpoint;
            break;
        }
        case ESQUIVAR_BLOQUE: {
            // --- NUEVO: Abortos por seguridad lateral y frontal ---
            if (distanciaIzquierdoFiltrada < UMBRAL_LATERAL_SAFETY_CM || distanciaDerechoFiltrada < UMBRAL_LATERAL_SAFETY_CM) {
                Serial.println(">>> ESQUIVAR: Lateral demasiado cerca, abortando y corrigiendo yaw.");
                setpoint = yawInicialManiobra;
                targetSetpoint = setpoint;
                integral = 0.0f; derivative = 0.0f; prevError = 0.0f;
                ultimoColorEsquivado = (seccionBlob >= 1 && seccionBlob <= 6) ? COLOR_ROJO : COLOR_VERDE;
                seccionBlob = 0; roiBlob = -1;
                estadoActual = NORMAL;
                subEstadoEsquivar = ES_NINGUNO;
                break;
            }
            if (distanciaCentroFiltrada < UMBRAL_FRONTAL_ABORT_ESQUIVE_CM) {
                Serial.println(">>> ESQUIVAR: Frontal demasiado cerca, abortando y corrigiendo yaw.");
                setpoint = yawInicialManiobra;
                targetSetpoint = setpoint;
                integral = 0.0f; derivative = 0.0f; prevError = 0.0f;
                ultimoColorEsquivado = (seccionBlob >= 1 && seccionBlob <= 6) ? COLOR_ROJO : COLOR_VERDE;
                seccionBlob = 0; roiBlob = -1;
                estadoActual = NORMAL;
                subEstadoEsquivar = ES_NINGUNO;
                break;
            }
             kp_actual = Kp_turn;
             ki_actual = Ki_turn;
             kd_actual = Kd_turn;
             setpoint_actual_yaw = setpointEsquive;
             break;
        }
        default:
            kp_actual = Kp; ki_actual = Ki; kd_actual = Kd;
            setpoint_actual_yaw = setpoint;
            break;
    }
    setpoint_actual_debug = setpoint_actual_yaw;
    error = calcularErrorAngular(setpoint_actual_yaw, currentYaw);
    bool enReversa = (estadoActual == CORRECCION_REVERSA) ||
                     (estadoActual == SALIR_PARKING && (subEstadoParking == SP_REVERSA_GIRANDO || subEstadoParking == SP_REVERSA_FINAL)) ||
                     (estadoActual == MANIOBRA_CARRIL_3 && subEstadoC3 == MC3_REVERSA_GIRANDO);
    if (enReversa) {
        if (fabs(error) < deadband_reversa) { error = 0.0f; }
    } else {
        if (fabs(error) < deadband) { error = 0.0f; }
    }
    integral += error * dt_global;
    integral = constrain(integral, -integralLimit, integralLimit);
    derivative = (dt_global > 0.0001f) ? (error - prevError) / dt_global : 0.0f;
    prevError = error;
    pidOutput = (kp_actual * error) + (ki_actual * integral) + (kd_actual * derivative);
    
    float comandoTotal = pidOutput;

    int servoCommand;
    if (estadoActual == SALIR_PARKING) {
        if (subEstadoParking == SP_REVERSA_GIRANDO) {
            servoCommand = (direccionPrimerGiro == DERECHA) ? servoMaxAngle : servoMinAngle;
        } else if (subEstadoParking == SP_AVANCE_SALIDA) {
            servoCommand = (direccionPrimerGiro == DERECHA) ? servoMinAngle : servoMaxAngle;
        } else if (subEstadoParking == SP_CORRIGIENDO_YAW) {
            servoCommand = servoNeutral - comandoTotal;
        } else if (subEstadoParking == SP_REVERSA_FINAL) {
            servoCommand = servoNeutral + comandoTotal;
        } else {
            servoCommand = servoNeutral;
        }
    } else if (enReversa) {
        servoCommand = servoNeutral + comandoTotal;
    } else {
        servoCommand = servoNeutral - comandoTotal;
    }
    // --- NUEVO: Sesgo de seguridad lateral global ---
    // Calcula cuánto nos estamos acercando a paredes laterales respecto al umbral.
    float deficitIzq = max(0.0f, UMBRAL_LATERAL_SAFETY_CM - distanciaIzquierdoFiltrada);
    float deficitDer = max(0.0f, UMBRAL_LATERAL_SAFETY_CM - distanciaDerechoFiltrada);
    float biasYaw = (deficitIzq - deficitDer) * YAW_SAFETY_BIAS_K; // positivo: empuja a la derecha
    biasYaw = constrain(biasYaw, -YAW_SAFETY_BIAS_MAX, YAW_SAFETY_BIAS_MAX);

    // Convertimos el bias de yaw a corrección en servo respetando la convención usada:
    // En avance: servo = neutral - pid; en reversa: neutral + pid.
    // Un bias positivo (derecha) debe disminuir el ángulo de servo en avance (hacia minAngle),
    // y aumentarlo en reversa.
    if (biasYaw != 0.0f) {
        if (enReversa) {
            servoCommand = servoCommand + (int)biasYaw;
        } else {
            servoCommand = servoCommand - (int)biasYaw;
        }
    }

    // Refuerzo temporal de pegado a pared durante recuperación
    if (pegadoAParedActivo && ladoPegado != NINGUNA) {
        int pegadoBias = (ladoPegado == IZQUIERDA) ? -(int)PEGADO_BIAS_DEG : (int)PEGADO_BIAS_DEG;
        if (enReversa) {
            servoCommand = servoCommand + pegadoBias;
        } else {
            servoCommand = servoCommand - pegadoBias;
        }
    }

    servoCommand = constrain(servoCommand, servoMinAngle, servoMaxAngle);
    steeringServo.write(servoCommand);
}
void manejarEstados() {
    EstadoRobot estadoAnterior = estadoActual;
    // --- NUEVO: Supervisor en sección de curva: frontal crítica -> estrategia de recuperación ---
    if (enSeccionDeCurva) {
        if (recSeccion == RC_NINGUNO && distanciaCentroFiltrada < UMBRAL_FRONTAL_SEGURIDAD_SECCION_CM) {
            Serial.println(">>> SECCION: Frontal crítico en sección. Iniciando recuperación.");
            recSeccion = RC_DETENIENDO;
            pausarConteoSeccion = true; // no contar distancia de sección durante la recuperación
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
                case RC_CORRIGE_YAW: {
                    // Realinear yaw al setpoint de la sección
                    float err = fabs(calcularErrorAngular(setpoint, currentYaw));
                    moverAdelante(VELOCIDAD_ALINEACION_YAW);
                    if (err < YAW_ALIGNED_TOLERANCE) {
                        detenerMotor();
                        // Evaluar pared opuesta al primer giro
                        float distOpuesta = (direccionPrimerGiro == DERECHA) ? distanciaIzquierdoFiltrada : distanciaDerechoFiltrada;
                        if (distOpuesta > LATERAL_OBJETIVO_PEGADO_CM) {
                            ladoPegado = (direccionPrimerGiro == DERECHA) ? IZQUIERDA : DERECHA;
                            pegadoAParedActivo = true;
                            recSeccion = RC_PEGARSE;
                            resetEncoder();
                        } else {
                            recSeccion = RC_FINALIZAR;
                        }
                    }
                    break; }
                case RC_PEGARSE: {
                    // Avanzar con pequeño sesgo hacia pared opuesta por una distancia moderada
                    moverAdelante(velocidadNormal);
                    // Aplica sesgo directo al servo vía setpoint de yaw: pequeño desplazamiento temporal
                    // Para no tocar PID, usamos el sesgo lateral global ya implementado + un refuerzo temporal ligero.
                    int refServo = 0;
                    if (ladoPegado == IZQUIERDA) refServo = -(int)PEGADO_BIAS_DEG; else if (ladoPegado == DERECHA) refServo = (int)PEGADO_BIAS_DEG;
                    // Aplicaremos el refuerzo cuando termine el ciclo de control (efecto acumulado por corto tramo)
                    // Condición de finalización: recorrer ~20 cm
                    if (fabs(leerDistanciaEncoder()) >= 20.0f) {
                        recSeccion = RC_FINALIZAR;
                        pegadoAParedActivo = false;
                        ladoPegado = NINGUNA;
                    }
                    break; }
                case RC_FINALIZAR:
                    // Terminar recuperación: reanudar conteo de sección y limpiar
                    pausarConteoSeccion = false;
                    recSeccion = RC_NINGUNO;
                    // Importante: bloquear re-entrada a ESQUIVAR_BLOQUE durante lo que reste de sección
                    bloquearEsquiveEnSeccion = true;
                    break;
                default:
                    break;
            }
        }
    }
    switch (estadoActual) {
        case SALIR_PARKING: {
            unsigned long tiempoEnSubEstado = millis() - tiempoInicioSubEstadoParking;
            switch (subEstadoParking) {
                case SP_INICIO:
                    detenerMotor();
                    Serial.println(">>> Iniciando maniobra de salida de parking...");
                    if (distanciaDerechoFiltrada > distanciaIzquierdoFiltrada) {
                        direccionPrimerGiro = DERECHA;
                    } else {
                        direccionPrimerGiro = IZQUIERDA;
                    }
                    Serial.print(">>> Dirección de salida determinada: ");
                    Serial.println(direccionPrimerGiro == DERECHA ? "DERECHA" : "IZQUIERDA");
                    tiempoInicioSubEstadoParking = millis();
                    subEstadoParking = SP_REVERSA_GIRANDO;
                    break;
                case SP_REVERSA_GIRANDO:
                    moverAtras(VELOCIDAD_SALIDA_PARKING);
                    if (tiempoEnSubEstado >= TIEMPO_REVERSA_INICIAL_MS) {
                        tiempoInicioSubEstadoParking = millis();
                        subEstadoParking = SP_AVANCE_SALIDA;
                    }
                    break;
                case SP_AVANCE_SALIDA:
                    moverAdelante(VELOCIDAD_SALIDA_PARKING);
                    if (tiempoEnSubEstado >= TIEMPO_AVANCE_SALIDA_MS) {
                        tiempoInicioSubEstadoParking = millis();
                        subEstadoParking = SP_CORRIGIENDO_YAW;
                    }
                    break;
                case SP_CORRIGIENDO_YAW: {
                    moverAdelante(VELOCIDAD_ALINEACION_YAW);
                    bool isAligned = fabs(currentYaw) < YAW_ALIGNED_TOLERANCE;
                    bool timeout = tiempoEnSubEstado > TIMEOUT_YAW_ALIGN_MS;
                    if (isAligned || timeout) {
                        if (timeout) Serial.println(">>> Timeout en alineacion de Yaw, continuando...");
                        else Serial.println(">>> Yaw alineado correctamente.");
                        integral = 0.0f;
                        derivative = 0.0f;
                        prevError = 0.0f;
                        tiempoInicioSubEstadoParking = millis();
                        subEstadoParking = SP_REVERSA_FINAL;
                    }
                    break;
                }
                case SP_REVERSA_FINAL:
                    moverAtras(VELOCIDAD_SALIDA_PARKING);
                     if (tiempoEnSubEstado >= TIEMPO_REVERSA_FINAL_MS) {
                        detenerMotor();
                        Serial.println(">>> Maniobra finalizada. Solicitando comando '2' a la camara...");
                        // Limpiar bandera antes de esperar nueva respuesta de CHECK
                        checkCompleto = false;
                        Serial1.write('2');
                        tiempoInicioSubEstadoParking = millis();
                        subEstadoParking = SP_ESPERANDO_CAMARA;
                    }
                    break;
                case SP_ESPERANDO_CAMARA:
                    detenerMotor();
                    if (checkCompleto) {
                        Serial.println(">>> Plan de CHECK recibido.");
                        checkCompleto = false; // Resetear flag
                        integral = 0.0f; derivative = 0.0f; prevError = 0.0f;
                        if (contadorCurvas >= NUM_CURVAS_ANTES_DE_PARAR) {
                            estadoActual = DETENIDO;
                        } else {
                            estadoActual = NORMAL;
                            tiempoUltimaManiobra = millis();
                            if (estadoAnterior == CORRECCION_REVERSA || estadoAnterior == MANIOBRA_CARRIL_3) {
                              Serial.println(">>> SECCION: Iniciando seccion de curva.");
                              enSeccionDeCurva = true;
                              encoderAlInicioDeCurva = leerDistanciaEncoder();
                              if (direccionPrimerGiro == DERECHA) {
                                 Serial1.write('R');
                                 Serial.println(">>> CMD: Mascara DERECHA (ignora izq)");
                              } else if (direccionPrimerGiro == IZQUIERDA) {
                                 Serial1.write('L');
                                 Serial.println(">>> CMD: Mascara IZQUIERDA (ignora der)");
                              }
                              mascaraActivada = true;
                              // Activar restriccion: si C,1 en el ultimo CHECK, prohibir carril 1 durante esta seccion
                              prohibirCarril1Seccion = marcasCarrilDetectadasUltimoCheck;
                            }
                        }
                    }
                    break;
            }
            break;
        }
        case NORMAL: {
             if (distanciaCentroFiltrada < UMBRAL_DETENCION_FRONTAL) {
                 detenerMotor();
             } else {
                moverAdelante(velocidadNormal);
                if (!enSeccionDeCurva) {
                   bool hayApertura = false;
                   if (direccionPrimerGiro == DERECHA) {
                      if (distanciaDerechoFiltrada > UMBRAL_APERTURA_LATERAL) {
                          hayApertura = true;
                      }
                   } else if (direccionPrimerGiro == IZQUIERDA) {
                      if (distanciaIzquierdoFiltrada > UMBRAL_APERTURA_LATERAL) {
                          hayApertura = true;
                      }
                   }
      
                   bool frontalCerca = distanciaCentroFiltrada < UMBRAL_DISTANCIA_FRONTAL_GIRO;
       
                   if (hayApertura && frontalCerca) {
                       detenerMotor();
                       Serial.println(">>> Condiciones de giro cumplidas. Confirmando pared...");
                       // Limpiar respuesta previa para evitar confirmaciones obsoletas
                       respuestaCamaraCheck = -1;
                       Serial1.write('3');
                       estadoActual = ESPERANDO_CONFIRMACION_PARED;
                   }
                }
             }
             break;
        }
        case EN_MANIOBRA: {
            unsigned long tiempoEnSubEstado = millis() - tiempoInicioSubEstadoManiobra;
            switch (subEstadoManiobra) {
                case M_INICIO:
                    Serial.println(">>> Maniobra C1/C2: Iniciando avance recto.");
                    moverAdelante(velocidadNormal);
                    tiempoInicioSubEstadoManiobra = millis();
                    subEstadoManiobra = M_AVANCE_RECTO;
                    break;
                
                case M_AVANCE_RECTO:
                    moverAdelante(velocidadNormal); // Mantiene el avance
                    if (distanciaCentroFiltrada < UMBRAL_FRONTAL_PARA_INICIAR_GIRO_CM) {
                        Serial.println(">>> Maniobra C1/C2: Frontal cerca. Iniciando giro.");
                        resetEncoder();
                        tiempoInicioSubEstadoManiobra = millis();
                        subEstadoManiobra = M_GIRO_EN_CURVA;
                    }
                    break;

                case M_GIRO_EN_CURVA: {
                    moverAdelante(velocidadGiro);
                    
                    // --- LÓGICA DE GIRO CORREGIDA: Basada en ángulo, no en distancia ---
                    float error_de_giro = calcularErrorAngular(targetSetpoint, currentYaw);
                    
                    // El giro se considera completo cuando el robot está alineado con el nuevo eje.
                    if (fabs(error_de_giro) < YAW_TOLERANCE_GIRO) {
                        Serial.println(">>> Maniobra C1/C2: Giro por ANGULO completado.");
                        detenerMotor();
                        setpoint = targetSetpoint;
                        integral = 0.0f;
                        derivative = 0.0f;
                        prevError = 0.0f;
                        
                        // Preparar para el siguiente estado
                        resetEncoder();
                        distanciaReversaObjetivo = (carrilActual == 1) ? DISTANCIA_REVERSA_C1_CM : DISTANCIA_REVERSA_C2_CM;
                        
                        iniciandoReversa = true; // Indicar que necesitamos detenernos primero
                        estadoActual = CORRECCION_REVERSA;
                    }
                    break;
                }
            }
            break;
        }
        case CORRECCION_REVERSA: {
            // --- LÓGICA DE TRANSICIÓN SEGURA A REVERSA ---
            if (iniciandoReversa) {
                detenerMotor(); // Primero, asegurar que el robot se detiene completamente
                if (velocidadActualMotor == 0) {
                    iniciandoReversa = false; // Una vez detenido, podemos proceder
                }
            } else {
                // Ahora que estamos detenidos, iniciar el movimiento hacia atrás
                moverAtras(velocidadGiro);
                
                // Comprobar si hemos completado la distancia de reversa
                if (fabs(leerDistanciaEncoder()) >= distanciaReversaObjetivo) {
                    detenerMotor();
                    Serial.println(">>> Maniobra C1/C2 finalizada. Solicitando CHECK...");
                    Serial1.write('1');
                    estadoActual = ESPERANDO_RESPUESTA_CAMARA;
                    iniciarSeccionTrasCheck = true;
                }
            }
            break;
        }
        case ESPERANDO_RESPUESTA_CAMARA: {
            detenerMotor();
            if (checkCompleto) {
                Serial.println(">>> Plan de CHECK recibido.");
                checkCompleto = false; // Resetear flag
                integral = 0.0f; derivative = 0.0f; prevError = 0.0f;
                if (contadorCurvas >= NUM_CURVAS_ANTES_DE_PARAR) {
                    estadoActual = DETENIDO;
                } else {
                    estadoActual = NORMAL;
                    tiempoUltimaManiobra = millis();
                    if (iniciarSeccionTrasCheck) {
                      Serial.println(">>> SECCION: Iniciando seccion de curva.");
                      enSeccionDeCurva = true;
                      encoderAlInicioDeCurva = leerDistanciaEncoder();
                      if (direccionPrimerGiro == DERECHA) {
                         Serial1.write('R');
                         Serial.println(">>> CMD: Mascara DERECHA (ignora izq)");
                      } else if (direccionPrimerGiro == IZQUIERDA) {
                         Serial1.write('L');
                         Serial.println(">>> CMD: Mascara IZQUIERDA (ignora der)");
                      }
                      mascaraActivada = true;
                      // Activar restriccion: si C,1 en el ultimo CHECK, prohibir carril 1 durante esta seccion
                      prohibirCarril1Seccion = marcasCarrilDetectadasUltimoCheck;
                      iniciarSeccionTrasCheck = false;
                    }
                }
            }
            break;
        }
        case ESPERANDO_CONFIRMACION_PARED: {
            detenerMotor();
            if (respuestaCamaraCheck != -1) {
                if (respuestaCamaraCheck == 9) { // Es una pared
                    Serial.println(">>> Pared confirmada. Iniciando maniobra.");
                    
                    if (direccionPrimerGiro != NINGUNA) {
                        float distanciaLateralRelevante = (direccionPrimerGiro == DERECHA) ? distanciaIzquierdoFiltrada : distanciaDerechoFiltrada;
                        if (distanciaLateralRelevante <= CARRIL_UMBRAL_1) {
                            // Respetar restriccion por marcas: no carril 1 en esta seccion
                            carrilActual = prohibirCarril1Seccion ? 2 : 1;
                        } else if (distanciaLateralRelevante <= CARRIL_UMBRAL_2) {
                            carrilActual = 2;
                        } else {
                            carrilActual = 3;
                        }
                    } else {
                        carrilActual = 0;
                    }

                    targetSetpoint = setpoint + (direccionPrimerGiro == IZQUIERDA ? -90.0f : 90.0f);
                    while (targetSetpoint <= -180.0f) targetSetpoint += 360.0f;
                    while (targetSetpoint > 180.0f) targetSetpoint -= 360.0f;
                    
                    integral = 0.0f; derivative = 0.0f;
                    prevError = calcularErrorAngular(targetSetpoint, currentYaw);
                    contadorCurvas++;
                    
                    if (carrilActual == 3) {
                        Serial.println(">>> CARRIL 3 DETECTADO. INICIANDO MANIOBRA ESPECIAL.");
                        estadoActual = MANIOBRA_CARRIL_3;
                        subEstadoC3 = MC3_INICIO;
                        tiempoInicioSubEstadoC3 = millis();
                    } else {
                        Serial.print(">>> Carril "); Serial.print(carrilActual); Serial.println(" detectado. Maniobra normal.");
                        tiempoInicioSubEstadoManiobra = millis();
                        estadoActual = EN_MANIOBRA;
                        subEstadoManiobra = M_INICIO;
                    }
                    respuestaCamaraCheck = -1;
                } else if (respuestaCamaraCheck == 8) { // No es una pared (bloque detectado)
                   Serial.println(">>> Bloque inesperado en curva. Cancelando giro para re-evaluar.");
                   estadoActual = NORMAL; // Volver a NORMAL para que la lógica de esquive se active
                   respuestaCamaraCheck = -1;
                }
            }
            break;
        }
        case MANIOBRA_CARRIL_3: {
            unsigned long tiempoEnSubEstadoC3 = millis() - tiempoInicioSubEstadoC3;
            switch (subEstadoC3) {
                case MC3_INICIO:
                    tiempoInicioSubEstadoC3 = millis();
                    subEstadoC3 = MC3_AVANCE_RECTO;
                    break;
                case MC3_AVANCE_RECTO: {
                    moverAdelante(velocidadNormal);
                    if (distanciaCentroFiltrada < UMBRAL_FRONTAL_PARA_INICIAR_GIRO_CM) {
                        Serial.println(">>> Maniobra C3: Frontal cerca. Iniciando reversa con giro.");
                        resetEncoder();
                        tiempoInicioSubEstadoC3 = millis();
                        subEstadoC3 = MC3_REVERSA_GIRANDO;
                    }
                    break;
                }
                case MC3_REVERSA_GIRANDO:
                    moverAtras(velocidadGiro);

                    // --- LÓGICA DE GIRO CORREGIDA: Basada en ángulo ---
                    float error_de_giro_c3 = calcularErrorAngular(targetSetpoint, currentYaw);

                    if (fabs(error_de_giro_c3) < YAW_TOLERANCE_GIRO) {
                        detenerMotor();
                        setpoint = targetSetpoint;

                        // --- CORRECCIÓN: Resetear el estado del PID es crucial ---
                        integral = 0.0f;
                        derivative = 0.0f;
                        prevError = 0.0f;

                        Serial.println(">>> Maniobra C3 finalizada. Solicitando CHECK...");
                        Serial1.write('1');
                        estadoActual = ESPERANDO_RESPUESTA_CAMARA;
                        iniciarSeccionTrasCheck = true;
                    }
                    break;
            }
            break;
        }
        case ESQUIVAR_BLOQUE: {
            // --- NUEVO: Supervisor de Yaw en Sección de Curva ---
            if (enSeccionDeCurva) {
                float errorConSetpointRecto = calcularErrorAngular(setpoint, currentYaw);
                if (fabs(errorConSetpointRecto) > YAW_TOLERANCE_SECCION_CURVA) {
                    Serial.println(">>> YAW EXCEDIDO EN SECCION CURVA DURANTE ESQUIVE. ABORTANDO Y CORRIGIENDO...");
                    setpointEsquive = setpoint; // Forzar al PID a volver al setpoint del tramo recto
                    subEstadoEsquivar = ES_NINGUNO;
                    seccionBlob = 0; // Limpiar datos para no re-activar
                    roiBlob = -1;
                    estadoActual = NORMAL;
                    break; // Salir del estado de esquive inmediatamente
                }
            }
            // --- FIN NUEVO ---
            // Velocidad adaptativa según distancia frontal (antichoques suave)
            int velAdapt = VEL_ESQUIVE_MIN;
            if (distanciaCentroFiltrada < 999.0f) {
                float d = distanciaCentroFiltrada;
                if (d < 15.0f) d = 15.0f;
                if (d > 60.0f) d = 60.0f;
                float t = (d - 15.0f) / (60.0f - 15.0f);
                velAdapt = (int)(VEL_ESQUIVE_MIN + t * (VEL_ESQUIVE_MAX - VEL_ESQUIVE_MIN));
            }
            moverAdelante(velAdapt);

            if (subEstadoEsquivar == ES_NINGUNO) { // Al entrar por primera vez
                Serial.println(">>> ESQUIVAR: Evaluando maniobra...");
                yawInicialManiobra = currentYaw;

                bool esRojo = (seccionBlob >= 1 && seccionBlob <= 6);
                colorEsquiveActual = esRojo ? COLOR_ROJO : COLOR_VERDE;
                bool necesitaEsquivar = true;
                
                // Asignar valores por defecto para un esquive normal
                anguloEsquiveActual = ANGULO_ESQUIVE;

                if (esRojo) {
                    direccionEsquive = IZQUIERDA; // Corregido: Rojo -> Esquivar a la Izquierda
                    Serial.print(">>> ESQUIVAR: Bloque ROJO detectado. ");
                    if (roiBlob == 0) { // Izquierda
                        Serial.println("ROI 0 (Izq), no se esquiva.");
                        necesitaEsquivar = false;
                    } else if (roiBlob == 1) { // Centro
                        Serial.println("ROI 1 (Centro), esquive normal a la IZQUIERDA.");
                    } else { // roiBlob == 2 o fallback
                        Serial.println("ROI 2 (Der), esquive FUERTE a la IZQUIERDA.");
                        anguloEsquiveActual = ANGULO_ESQUIVE_FUERTE;
                    }
                } else { // Es Verde
                    direccionEsquive = DERECHA; // Corregido: Verde -> Esquivar a la Derecha
                    Serial.print(">>> ESQUIVAR: Bloque VERDE detectado. ");
                    if (roiBlob == 2) { // Derecha
                        Serial.println("ROI 2 (Der), no se esquiva.");
                        necesitaEsquivar = false;
                    } else if (roiBlob == 1) { // Centro
                        Serial.println("ROI 1 (Centro), esquive normal a la DERECHA.");
                    } else { // roiBlob == 0 o fallback
                        Serial.println("ROI 0 (Izq), esquive FUERTE a la DERECHA.");
                        anguloEsquiveActual = ANGULO_ESQUIVE_FUERTE;
                    }
                }

                if (necesitaEsquivar) {
                    Serial.print(">>> Iniciando esquive. Angulo: "); Serial.println(anguloEsquiveActual);
                    
                    if (direccionEsquive == DERECHA) {
                        setpointEsquive = yawInicialManiobra - anguloEsquiveActual;
                    } else { // IZQUIERDA
                        setpointEsquive = yawInicialManiobra + anguloEsquiveActual;
                    }
                    subEstadoEsquivar = ES_GIRO_INICIAL;
                } else {
                    Serial.println(">>> Maniobra de esquive no necesaria. Volviendo a NORMAL.");
                    seccionBlob = 0; // Limpiar para no re-activar
                    roiBlob = -1;
                    estadoActual = NORMAL;
                    subEstadoEsquivar = ES_NINGUNO;
                }
            }

            // Transiciones de la maniobra
            switch (subEstadoEsquivar) {
                case ES_GIRO_INICIAL: {
                    float error_esquive = calcularErrorAngular(setpointEsquive, currentYaw);
                    if (fabs(error_esquive) < TOLERANCIA_ANGULO_ESQUIVE) {
                        Serial.println(">>> ESQUIVAR: Giro inicial completo. Avanzando en paralelo.");
                        // Mantener el setpointEsquive para avanzar en el nuevo angulo
                        subEstadoEsquivar = ES_AVANCE_PARALELO;
                    }
                    break;
                }
                case ES_AVANCE_PARALELO: {
                    bool pasilloLibre = (distanciaCentroFiltrada >= UMBRAL_PASILLO_LIBRE_CM);
                    bool sinBlob = (seccionBlob == 0);
                    if (pasilloLibre || sinBlob) {
                        Serial.println(">>> ESQUIVAR: Pasillo libre detectado. Regresando al carril.");
                        // Apuntar de vuelta al ángulo original para realinear
                        setpointEsquive = yawInicialManiobra;
                        subEstadoEsquivar = ES_GIRO_REGRESO;
                    }
                    break;
                }
                case ES_GIRO_REGRESO: {
                    float error_regreso = calcularErrorAngular(yawInicialManiobra, currentYaw);
                    bool anguloAlcanzado = fabs(error_regreso) < TOLERANCIA_ANGULO_ESQUIVE;

                    if (anguloAlcanzado) {
                        Serial.println(">>> ESQUIVAR: Maniobra completada. Volviendo a estado NORMAL.");
                        setpoint = yawInicialManiobra; // Restaurar setpoint original
                        targetSetpoint = setpoint;
                        integral = 0.0f; // Resetear PID
                        seccionBlob = 0; // Limpiar datos de blob para no re-activar
                        roiBlob = -1;
                        ultimoColorEsquivado = colorEsquiveActual;
                        colorEsquiveActual = COLOR_NINGUNO;
                        estadoActual = NORMAL;
                        subEstadoEsquivar = ES_NINGUNO; // Resetear sub-estado
                    }
                    // El PID ya trabaja hacia yawInicialManiobra
                    break;
                }
            }
            break;
        }
        case DETENIDO: {
             detenerMotor();
             break;
        }
    }
    if (estadoActual != estadoAnterior) {
        Serial.print(">>> Cambio de estado: ");
        Serial.print(estadoNombres[estadoAnterior]);
        Serial.print(" -> ");
        Serial.println(estadoNombres[estadoActual]);
    }
}

volatile long contadorPulsos = 0;

void IRAM_ATTR isr_encoder() {
  // Lee el estado actual del pin B para determinar la dirección.
  // Si al avanzar los pulsos son negativos, se deben invertir el ++ y --.
  if (digitalRead(ENCODER_PIN_B)) {
    contadorPulsos--;
  } else {
    contadorPulsos++;
  }
}

void setupEncoder() {
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  
  // Configura la interrupción para que llame a 'isr_encoder'
  // cada vez que el pin A tenga un borde de subida (RISING).
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), isr_encoder, RISING);
  
  Serial.println("Encoder de cuadratura inicializado.");
}

void resetEncoder() {
  // Se desactivan las interrupciones para asegurar una escritura atómica.
  noInterrupts();
  contadorPulsos = 0;
  interrupts();
}

float leerDistanciaEncoder() {
  long pulsos_copia;
  
  // Desactivar interrupciones, copiar el valor y reactivarlas.
  // Esto asegura que leemos un valor completo y no uno corrupto.
  noInterrupts();
  pulsos_copia = contadorPulsos;
  interrupts();

  // Usa la fórmula de calibración simple.
  float distancia_cm = (float)pulsos_copia / PULSOS_POR_CM;
  
  return distancia_cm;
}