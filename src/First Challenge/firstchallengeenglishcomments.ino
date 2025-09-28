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

// ==================== CONFIGURABLE PARAMETERS ====================
// IMU and I2C bus configuration
const uint8_t I2C_SDA_PIN = A4;  // Current user wiring configuration
const uint8_t I2C_SCL_PIN = A5;    // I2C clock line pin
const uint8_t BNO08X_INT_PIN = 2;  // BNO080 interrupt pin for data ready signal
const uint8_t BNO08X_RST_PIN = 3;  // BNO080 reset pin for hardware reset capability
const uint8_t BNO08X_ADDR = 0x4B;  // I2C address of BNO080 sensor

// Motor driver and auxiliary hardware pins
const int pwma = 9;           // PWM control pin for motor speed
const int ain2 = 10;          // Motor direction control pin (L298N)
const int ain1 = 11;          // Motor direction control pin (L298N)
const int stby = 12;          // Standby pin to enable/disable motor driver
const int startButtonPin = A0; // Start button input pin

// Servo motor configuration
const int servoPin = 8;        // Servo control pin
const int servoNeutral = 90;    // Neutral position (straight ahead)
const int servoMinAngle = 10;   // Minimum servo angle limit
const int servoMaxAngle = 170;  // Maximum servo angle limit

// Ultrasonic sensor configuration
#define TRIG_IZQUIERDO 5    // Left ultrasonic trigger pin
#define ECHO_IZQUIERDO 4     // Left ultrasonic echo pin
#define TRIG_CENTRO A2       // Center ultrasonic trigger pin
#define ECHO_CENTRO A1       // Center ultrasonic echo pin
#define TRIG_DERECHA 6       // Right ultrasonic trigger pin
#define ECHO_DERECHA 7       // Right ultrasonic echo pin
const unsigned int MAX_US_DISTANCE_CM = 200;     // Maximum measurable distance
const long TIMEOUT_ULTRASONICO = 20000;          // Timeout for ultrasonic readings
const float DISTANCIA_MAX_VALIDA = 500.0;        // Maximum valid distance reading
const float DISTANCIA_MIN_VALIDA = 1.0;          // Minimum valid distance reading
const float umbralRatioLateral = 0.6;            // Lateral threshold ratio for opening detection
const float umbralRatioCentral = 1.3;            // Central threshold ratio for wall detection
const unsigned long MIN_INTERVALO_PING_MS = 15; // Minimum interval between ultrasonic pings

// PID controller parameters for yaw control
float Kp = 1.5, Ki = 0.01, Kd = 0.2;        // Normal navigation PID gains
float Kp_turn = 0.5, Ki_turn = 0.25, Kd_turn = 1.00;  // Turning maneuver PID gains
float Kp_stab = 1.02, Ki_stab = 0.03, Kd_stab = 1.00; // Stabilization PID gains
const unsigned long sampleInterval = 20;       // PID sample time in microseconds
const float integralLimit = 40.0;              // Integral windup protection limit
const float deadband = 0.5;                    // Deadband for yaw error to prevent oscillation

// Maneuver timing and control parameters
const unsigned long TIMEOUT_MANIOBRA = 4000;           // Maximum time for turn maneuver
const float TURN_ANGLE_TOLERANCE = 2;                   // Angle tolerance for turn completion
const unsigned long tiempoEstabilizacion = 300;        // Stabilization phase duration
const unsigned long tiempoCorreccion = 400;            // Correction phase duration
const unsigned long TIMEOUT_CORRECCION = 800;          // Maximum correction phase timeout
const float CORRECTION_ANGLE_TOLERANCE = 5;             // Yaw stability tolerance for correction
const int NUM_CURVAS_ANTES_DE_PARAR = 12;              // Number of turns before parking sequence

// Motor speed settings for different phases
const int velocidadNormal = 255;        // Normal navigation speed
const int velocidadGiro = 255;          // Turning maneuver speed
const int velocidadEstabilizacion = 255; // Stabilization phase speed
const int velocidadCorreccion = 255;     // Correction phase speed
const int velocidadCorreccionMin = 180;  // Minimum correction speed for ramping
const int velocidadCorreccionMax = 255;  // Maximum correction speed for ramping

// Parking algorithm parameters
float Kp_parking_lateral = 3.0;                              // Lateral positioning PID proportional gain
float Kd_parking_lateral = 0.4;                              // Lateral positioning PID derivative gain
const float DEADBAND_PARKING_LATERAL = 5.0;                  // Deadband for lateral positioning
const float DISTANCIA_CENTRAL_OBJETIVO_TOLERANCIA_PARKING = 5.0; // Front distance tolerance for parking
const int VELOCIDAD_PARKING_AVANCE = 105;                    // Forward speed during parking
const int VELOCIDAD_PARKING_RETROCESO = 105;                 // Reverse speed during parking
const int VELOCIDAD_PARKING_ALINEACION_LATERAL = 105;        // Lateral alignment speed
const unsigned long TIMEOUT_PARKING_SUBFASE = 10000;        // Timeout for parking subphases
const unsigned long intervalo = 1000;                        // Interval for alternating parking attempts

// Signal filtering parameters
#define FILTER_SIZE 1  // Size of median filter window for ultrasonic readings
// ================= END OF CONFIGURABLE PARAMETERS =================

#if USE_NEWPING
// Limiting maximum distance reduces echo wait time and speeds up each measurement
NewPing sonarIzq(TRIG_IZQUIERDO, ECHO_IZQUIERDO, MAX_US_DISTANCE_CM);
NewPing sonarCen(TRIG_CENTRO,    ECHO_CENTRO,    MAX_US_DISTANCE_CM);
NewPing sonarDer(TRIG_DERECHA,   ECHO_DERECHA,   MAX_US_DISTANCE_CM);
// --- Asynchronous scheduler for NewPing (one ping at a time, non-blocking) ---
enum SensorId { SEN_CEN = 0, SEN_IZQ = 1, SEN_DER = 2 };  // Sensor identification enum
volatile unsigned int echoUs[3] = {0, 0, 0};            // Echo timing results in microseconds
volatile bool echoListo[3] = {false, false, false};     // Echo completion flags
volatile bool pingEnCurso = false;                      // Current ping operation flag
static SensorId sensorEnCurso = SEN_CEN;                // Currently active sensor

void echoCheckCen() {
  if (sonarCen.check_timer()) {
    echoUs[SEN_CEN] = sonarCen.ping_result;  // Store center sensor echo timing
    echoListo[SEN_CEN] = true;               // Mark center echo as ready
    pingEnCurso = false;                     // Free up ping scheduler
  }
}
void echoCheckIzq() {
  if (sonarIzq.check_timer()) {
    echoUs[SEN_IZQ] = sonarIzq.ping_result;  // Store left sensor echo timing
    echoListo[SEN_IZQ] = true;               // Mark left echo as ready
    pingEnCurso = false;                     // Free up ping scheduler
  }
}
void echoCheckDer() {
  if (sonarDer.check_timer()) {
    echoUs[SEN_DER] = sonarDer.ping_result;  // Store right sensor echo timing
    echoListo[SEN_DER] = true;               // Mark right echo as ready
    pingEnCurso = false;                     // Free up ping scheduler
  }
}

void iniciarPing(SensorId s) {
  if (pingEnCurso) return;  // Prevent overlapping ping operations
  switch (s) {
    case SEN_CEN: sonarCen.ping_timer(echoCheckCen); break;  // Start center sensor ping
    case SEN_IZQ: sonarIzq.ping_timer(echoCheckIzq); break;  // Start left sensor ping
    case SEN_DER: sonarDer.ping_timer(echoCheckDer); break;  // Start right sensor ping
  }
  sensorEnCurso = s;     // Track which sensor is currently active
  pingEnCurso = true;    // Mark ping operation as in progress
}

void servicioUltrasonidos() {
  // Fair scheduling pattern: CEN -> IZQ -> CEN -> DER -> ...
  static uint8_t fase = 0; // 0=CEN,1=IZQ,2=CEN,3=DER
  unsigned long ahoraMs = millis();
  if (pingEnCurso) return;  // Skip if ping already in progress

  // Check minimum interval constraints for each sensor
  bool puedeCen = (ahoraMs - ultimoPingCenMs) >= MIN_INTERVALO_PING_MS;
  bool puedeIzq = (ahoraMs - ultimoPingIzqMs) >= MIN_INTERVALO_PING_MS;
  bool puedeDer = (ahoraMs - ultimoPingDerMs) >= MIN_INTERVALO_PING_MS;

  // Try to find next available sensor in fair rotation pattern
  for (uint8_t intentos = 0; intentos < 4; intentos++) {
    uint8_t f = (fase + intentos) % 4;
    if ((f == 0 || f == 2) && puedeCen) {  // Center sensor slots (0 and 2)
      iniciarPing(SEN_CEN);
      ultimoPingCenMs = ahoraMs;
      fase = (f + 1) % 4;
      return;
    }
    if (f == 1 && puedeIzq) {  // Left sensor slot (1)
      iniciarPing(SEN_IZQ);
      ultimoPingIzqMs = ahoraMs;
      fase = (f + 1) % 4;
      return;
    }
    if (f == 3 && puedeDer) {  // Right sensor slot (3)
      iniciarPing(SEN_DER);
      ultimoPingDerMs = ahoraMs;
      fase = (f + 1) % 4;
      return;
    }
  }
}
#endif

BNO080 myIMU;        // IMU sensor object for orientation data
Servo steeringServo;  // Servo object for steering control

float yawOffset = 0.0f;  // Yaw offset for relative orientation
bool offsetSet = false;   // Flag indicating if yaw offset has been calibrated

float setpoint = 0.0, targetSetpoint = 0.0;  // Current and target yaw setpoints
float error = 0.0, prevError = 0.0;           // Current and previous yaw errors
float integral = 0.0, derivative = 0.0, pidOutput = 0.0;  // PID controller terms
unsigned long prevTimePID = 0;                // Previous PID calculation timestamp
float setpoint_actual_debug = 0.0;           // Debug variable for setpoint tracking
float currentYaw = 0.0f;                     // Current relative yaw angle
float dt_global = 0.02f;                     // Global delta time for PID calculations

// --- Motor speed ramping variables ---
int velocidadActualMotor = 0;      // Current motor speed
int velocidadObjetivoMotor = 0;    // Target motor speed
enum DireccionMotor { MOTOR_DETENIDO, MOTOR_ADELANTE, MOTOR_ATRAS };  // Motor direction states
DireccionMotor direccionMotor = MOTOR_DETENIDO;  // Current motor direction


enum EstadoRobot { NORMAL, EN_MANIOBRA, ESTABILIZANDO, CORRIGIENDO, PARKING, DETENIDO };  // Robot state machine
EstadoRobot estadoActual = NORMAL;  // Current robot state
const char* estadoNombres[] = {"NORMAL", "EN_MANIOBRA", "ESTABILIZANDO", "CORRIGIENDO", "PARKING", "DETENIDO"};  // State names for debugging

unsigned int contadorCurvas = 0;                    // Counter for completed turns
enum DireccionGiro { NINGUNA, IZQUIERDA, DERECHA };  // Turn direction enumeration
DireccionGiro direccionPrimerGiro = NINGUNA;         // Direction of first turn (for consistency)
unsigned long tiempoInicioManiobra = 0;              // Turn maneuver start timestamp
unsigned long estabilizacionInicio = 0;              // Stabilization phase start timestamp
unsigned long correccionInicio = 0;                  // Correction phase start timestamp
float distanciaCentralInicial = 0.0;                 // Initial center distance reference

float lecturasIzquierdo[FILTER_SIZE], lecturasCentro[FILTER_SIZE], lecturasDerecho[FILTER_SIZE];  // Ultrasonic reading buffers
int indiceLecturaIzquierdo = 0, indiceLecturaCentro = 0, indiceLecturaDerecho = 0;  // Buffer indices
float distanciaIzquierdoFiltrada = 0.0, distanciaCentroFiltrada = 0.0, distanciaDerechoFiltrada = 0.0;  // Filtered distances
float distanciaUltrasonicoCentro_dbg = 999.9;  // Debug variable for center distance

enum SubEstadoParking { SUB_PARKING_NO_INICIADO, SUB_PARKING_ALINEACION_LATERAL, SUB_PARKING_ALINEACION_FRONTAL, SUB_PARKING_FINALIZADO };  // Parking sub-states
SubEstadoParking subEstadoParkingActual = SUB_PARKING_NO_INICIADO;  // Current parking sub-state
bool paredObjetivoParkingEsIzquierda = false;  // Which side wall to align with
float errorParkLat = 0.0, prevErrorParkLat = 0.0, pdOutputParkLat = 0.0;  // Lateral parking PID terms
unsigned long tiempoInicioSubEstadoParking = 0, tiempoInicioIntentoActual = 0;  // Parking timing variables
float distanciaIzquierdoInicial = 50.0, distanciaDerechoInicial = 50.0;  // Initial lateral distances
bool intentandoAlineacionLateralActual = true;  // Flag for alternating lateral alignment attempts

volatile bool imuDataReady = false;  // IMU data ready flag
unsigned long ultimoTiempoDatosImu = 0;  // Last IMU data timestamp
// Last ping time per sensor (to avoid readings too close together)
unsigned long ultimoPingIzqMs = 0;  // Last left sensor ping time
unsigned long ultimoPingCenMs = 0;  // Last center sensor ping time
unsigned long ultimoPingDerMs = 0;  // Last right sensor ping time


// --- Robot Functions ---

void setReports(void) {
  Serial.println("Enabling Rotation Vector...");
  myIMU.enableRotationVector(50);  // Enable rotation vector reports at 50Hz
  Serial.println(F("Rotation Vector enabled."));
}

bool iniciarIMURobusto() {
  const uint8_t posiblesDirecciones[2] = {0x4B, 0x4A};  // Possible I2C addresses for BNO080
  // Add slower speeds to attempt communication stabilization without pull-ups
  const uint32_t clocks[4] = {100000, 50000, 20000, 400000};  // I2C clock speeds to try 
  Serial.print("INT level before init: "); Serial.println(digitalRead(BNO08X_INT_PIN));
  for (uint8_t ci = 0; ci < 4; ci++) { // Updated loop limit to 4
    Wire.setClock(clocks[ci]);
    for (uint8_t ai = 0; ai < 2; ai++) {
      uint8_t addr = posiblesDirecciones[ai];
      Serial.print("Trying IMU at 0x"); Serial.print(addr, HEX);
      Serial.print(" @I2C"); Serial.print(clocks[ci]/1000); Serial.print("k without INT/RST...");
      if (myIMU.begin(addr, Wire)) {
        Serial.println("OK");
        return true;
      }
      Serial.println("fail");
      Serial.print("INT level after attempt without INT: "); Serial.println(digitalRead(BNO08X_INT_PIN));
    }
  }
  Serial.println("Scanning I2C bus...");
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at 0x");
      Serial.println(address, HEX);
    }
  }
  return false;
}

// (Removed: custom gyroscope reset/calibration logic)

// (Removed: sample-based calibration routine; offset is set on first reading)


float medirDistancia(int trigPin, int echoPin) {
#if USE_NEWPING
    unsigned int cm = 0;
    if (trigPin == TRIG_IZQUIERDO && echoPin == ECHO_IZQUIERDO) {
        cm = sonarIzq.ping_cm();  // Get left sensor distance
    } else if (trigPin == TRIG_CENTRO && echoPin == ECHO_CENTRO) {
        cm = sonarCen.ping_cm();  // Get center sensor distance
    } else if (trigPin == TRIG_DERECHA && echoPin == ECHO_DERECHA) {
        cm = sonarDer.ping_cm();  // Get right sensor distance
    } else {
        cm = 0;  // Invalid pin combination
    }
    if (cm == 0) return 999.9;  // Return invalid distance if no echo
    float dist = (float)cm;
    if (dist < DISTANCIA_MIN_VALIDA || dist > DISTANCIA_MAX_VALIDA) return 999.9;  // Validate distance range
    return dist;
#else
    digitalWrite(trigPin, LOW); delayMicroseconds(2);     // Clear trigger pin
    digitalWrite(trigPin, HIGH); delayMicroseconds(10);   // Send 10us trigger pulse
    digitalWrite(trigPin, LOW);                           // Clear trigger pin
    long duration = pulseIn(echoPin, HIGH, TIMEOUT_ULTRASONICO);  // Measure echo duration
    if (duration == 0) return 999.9;  // Return invalid if timeout
    float dist = (duration * 0.0343) / 2.0;  // Convert to distance (cm)
    if (dist < DISTANCIA_MIN_VALIDA || dist > DISTANCIA_MAX_VALIDA) return 999.9;  // Validate range
    return dist;
#endif
}

int compareFloat(const void* a, const void* b) {
    float fa = *(const float*) a; float fb = *(const float*) b;  // Cast void pointers to float
    return (fa > fb) - (fa < fb);  // Return comparison result for sorting
}

float calcularMediana(float arr[], int n) {
    if (n <= 0) return 999.9;  // Invalid array size
    if (n == 1) return arr[0];  // Single element case
    float tempArr[n]; memcpy(tempArr, arr, n * sizeof(float));  // Create working copy
    qsort(tempArr, n, sizeof(float), compareFloat);  // Sort array
    int invalidos = 0; for(int i=0; i<n; i++) { if(tempArr[i] >= 999.0) invalidos++; }  // Count invalid readings
    if(invalidos > n / 2) return 999.9;  // Too many invalid readings
    int primerValidoIdx = 0;
    while(primerValidoIdx < n && tempArr[primerValidoIdx] >= 999.0) { primerValidoIdx++; }  // Find first valid reading
    int medianaIdx = primerValidoIdx + (n - invalidos - 1) / 2;  // Calculate median index of valid readings
    if(medianaIdx < 0) medianaIdx = 0;  // Bounds checking
     if (medianaIdx >= n) medianaIdx = n - 1;  // Bounds checking
    return tempArr[medianaIdx];  // Return median value
}

void moverAdelante(int velocidad) {
    direccionMotor = MOTOR_ADELANTE;  // Set motor direction to forward
    velocidadObjetivoMotor = constrain(velocidad, 0, 255);  // Constrain speed to valid range
}

void detenerMotor() {
    direccionMotor = MOTOR_DETENIDO;  // Set motor direction to stopped
    velocidadObjetivoMotor = 0;       // Set target speed to zero
}

float calcularErrorAngular(float target, float current) {
    float diff = target - current;  // Calculate raw difference
    while (diff <= -180.0) diff += 360.0;  // Normalize to [-180, 180] range
    while (diff > 180.0) diff -= 360.0;    // Normalize to [-180, 180] range
    return diff;  // Return normalized angular error
}

void moverAtras(int velocidad) {
    direccionMotor = MOTOR_ATRAS;  // Set motor direction to reverse
    velocidadObjetivoMotor = constrain(velocidad, 0, 255);  // Constrain speed to valid range
}

void setup() {
    Serial.begin(115200);  // Initialize serial communication
    unsigned long inicioEsperaSerial = millis();  // Start serial wait timer
    while (!Serial && millis() - inicioEsperaSerial < 2000) {  // Wait for serial connection or timeout
        delay(10);
    }
    pinMode(BNO08X_INT_PIN, INPUT_PULLUP);  // Configure INT pin with pullup
    pinMode(BNO08X_RST_PIN, OUTPUT);        // Configure reset pin as output
    digitalWrite(BNO08X_RST_PIN, HIGH);     // Keep reset pin high (not reset)
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);   // Initialize I2C with custom pins
    Wire.setClock(400000);                  // Set I2C clock to 400kHz
    Serial.println("1rondaV2.0");          // Print version info

    steeringServo.attach(servoPin);    // Attach servo to control pin
    steeringServo.write(servoNeutral);  // Set servo to neutral position

    if (!iniciarIMURobusto()) {
        Serial.println(F("CRITICAL FAILURE: BNO080 not detected. Check connections."));
        while (1);  // Halt execution if IMU not found
    }
    Serial.println(F("BNO080 found."));
    delay(100);     // Allow IMU to stabilize
    setReports();   // Configure IMU reports
    
    ultimoTiempoDatosImu = millis(); // Initialize watchdog timer

    pinMode(TRIG_IZQUIERDO, OUTPUT); pinMode(ECHO_IZQUIERDO, INPUT);  // Configure left ultrasonic pins
    pinMode(TRIG_CENTRO, OUTPUT); pinMode(ECHO_CENTRO, INPUT);       // Configure center ultrasonic pins
    pinMode(TRIG_DERECHA, OUTPUT); pinMode(ECHO_DERECHA, INPUT);     // Configure right ultrasonic pins

    Serial.println("Filling initial filter buffers...");
    for (int i = 0; i < FILTER_SIZE; i++) {
        lecturasIzquierdo[i] = medirDistancia(TRIG_IZQUIERDO, ECHO_IZQUIERDO); delay(15);  // Fill left buffer
        lecturasCentro[i] = medirDistancia(TRIG_CENTRO, ECHO_CENTRO); delay(5);             // Fill center buffer
        lecturasDerecho[i] = medirDistancia(TRIG_DERECHA, ECHO_DERECHA); delay(15);        // Fill right buffer
    }
    distanciaIzquierdoFiltrada = calcularMediana(lecturasIzquierdo, FILTER_SIZE);  // Initialize filtered distances
    distanciaCentroFiltrada = calcularMediana(lecturasCentro, FILTER_SIZE);
    distanciaDerechoFiltrada = calcularMediana(lecturasDerecho, FILTER_SIZE);
    
    Serial.println("-> Using ultrasonic for initial central distance.");
    distanciaCentralInicial = (distanciaCentroFiltrada < 999.0) ? distanciaCentroFiltrada : 100.0;  // Set reference distance
    
    distanciaIzquierdoInicial = (distanciaIzquierdoFiltrada < 999.0) ? distanciaIzquierdoFiltrada : 25.0;  // Set left reference
    distanciaDerechoInicial = (distanciaDerechoFiltrada < 999.0) ? distanciaDerechoFiltrada : 25.0;   // Set right reference

    Serial.print("Initial central distance (effective): "); Serial.println(distanciaCentralInicial);
    
    pinMode(pwma, OUTPUT); pinMode(ain2, OUTPUT); pinMode(ain1, OUTPUT); pinMode(stby, OUTPUT);  // Configure motor pins
    digitalWrite(stby, HIGH);  // Enable motor driver
    detenerMotor();            // Initialize motor as stopped

    currentYaw = 0.0f;        // Initialize yaw angle
    setpoint = 0.0f;          // Initialize setpoint
    targetSetpoint = 0.0;     // Initialize target setpoint
    prevTimePID = micros();   // Initialize PID timing
    estadoActual = NORMAL;    // Set initial robot state

    pinMode(startButtonPin, INPUT_PULLUP);  // Configure start button pin
    Serial.println("IMU ready. Press button to start...");
    while (digitalRead(startButtonPin) == HIGH) {  // Wait for button press
        delay(50);
    }
    Serial.println("Button pressed. Starting movement!");
    
    // PID soft-start: stabilize with neutral servo before enabling control
    moverAdelante(velocidadEstabilizacion);  // Start with stabilization speed
    estabilizacionInicio = millis();         // Start stabilization timer
    estadoActual = ESTABILIZANDO;            // Set to stabilization state
    integral = 0.0f; derivative = 0.0f; prevError = 0.0f;  // Reset PID terms
    Serial.println("System ready. Initial state: ESTABILIZANDO.");
}

// (Removed: reset handling with recalibration; reports and offset are reconfigured)

void actualizarMotorConRampa() {
    static unsigned long ultimoTiempoRampa = 0;      // Last ramp update time
    static DireccionMotor direccionRealMotor = MOTOR_DETENIDO;  // Actual motor direction
    const unsigned long INTERVALO_RAMPA_MS = 20;    // Ramp update interval
    const int ACELERACION = 15;                     // Acceleration rate per update
    const int DECELERACION = 25;                    // Deceleration rate per update

    unsigned long ahora = millis();
    if (ahora - ultimoTiempoRampa < INTERVALO_RAMPA_MS) {
        return; // Not time to update yet
    }
    ultimoTiempoRampa = ahora;

    // If we are stopped, we can accept the new direction immediately.
    if (velocidadActualMotor == 0) {
        direccionRealMotor = direccionMotor;
    }

    int objetivoActual = velocidadObjetivoMotor;
    // If a direction change is requested while moving, the priority objective is to stop.
    if (direccionMotor != direccionRealMotor && velocidadActualMotor > 0) {
        objetivoActual = 0;
    }

    // Acceleration/deceleration ramp logic
    if (velocidadActualMotor < objetivoActual) {
        velocidadActualMotor = min(objetivoActual, velocidadActualMotor + ACELERACION);  // Accelerate towards target
    } else if (velocidadActualMotor > objetivoActual) {
        velocidadActualMotor = max(objetivoActual, velocidadActualMotor - DECELERACION);  // Decelerate towards target
    }

    // Apply power and direction to H-Bridge
    switch (direccionRealMotor) {
        case MOTOR_ADELANTE:
            digitalWrite(ain2, LOW); digitalWrite(ain1, HIGH);  // Set forward direction
            analogWrite(pwma, velocidadActualMotor);            // Apply forward speed
            break;
        case MOTOR_ATRAS:
            digitalWrite(ain1, LOW); digitalWrite(ain2, HIGH);  // Set reverse direction
            analogWrite(pwma, velocidadActualMotor);            // Apply reverse speed
            break;
        case MOTOR_DETENIDO:
            digitalWrite(ain1, LOW); digitalWrite(ain2, LOW);   // Disable motor direction
            analogWrite(pwma, 0);                               // Set speed to zero
            velocidadActualMotor = 0; // Ensure speed is 0
            break;
    }
}


void loop() {
    if (estadoActual == DETENIDO) {
      detenerMotor(); // Ensure target is 0
      actualizarMotorConRampa();
      return; 
    }

    actualizarMotorConRampa();

    if (myIMU.hasReset()) {
      Serial.print("Sensor reset. Reconfiguring... ");
      setReports();   // Reconfigure IMU reports after reset
      offsetSet = false;  // Reset offset flag for recalibration
    }

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
        if (!offsetSet) { yawOffset = currentYawDegrees; offsetSet = true; }  // Set initial offset
        float relativeYaw = currentYawDegrees - yawOffset;  // Calculate relative yaw
        if (relativeYaw > 180.0) { relativeYaw -= 360.0; }    // Normalize to [-180, 180]
        else if (relativeYaw < -180.0) { relativeYaw += 360.0; }
        currentYaw = -relativeYaw;  // Invert for robot coordinate system
    }
    
    bool leerSensoresAhora = false;
    if (estadoActual == NORMAL || estadoActual == PARKING || estadoActual == CORRIGIENDO) {
        leerSensoresAhora = true;  // Enable sensor reading for these states
    }

    if (leerSensoresAhora) {
#if USE_NEWPING
        servicioUltrasonidos();  // Handle asynchronous ultrasonic readings
#endif
        actualizarFiltrosSensores();  // Update sensor filters
    }

    actualizarControl();  // Update PID control and servo
    manejarEstados();     // Handle state machine logic

    static unsigned long lastPrintTime = 0;
    if (millis() - lastPrintTime > 300) {  // Print debug info every 300ms
        Serial.print("estado:"); Serial.print(estadoNombres[estadoActual]);  // Current state
        Serial.print("|Yaw:"); Serial.print(currentYaw, 1);                  // Current yaw angle
        Serial.print("|SetP:"); Serial.print(setpoint_actual_debug, 1);      // Current setpoint
        Serial.print("|Err:"); Serial.print(error, 1);                       // Current error
        Serial.print("|US_I:"); Serial.print(distanciaIzquierdoFiltrada, 0);  // Left ultrasonic
        Serial.print("|US_C:"); Serial.print(distanciaUltrasonicoCentro_dbg, 0);  // Center ultrasonic
        Serial.print("|US_D:"); Serial.print(distanciaDerechoFiltrada, 0);   // Right ultrasonic
        Serial.print("|curv:"); Serial.print(contadorCurvas);                // Turn counter
        Serial.print("|Srv:"); Serial.print(steeringServo.read());           // Servo position
        Serial.println();
        lastPrintTime = millis();
    }
}

void actualizarFiltrosSensores() {
     static bool leerIzquierdaEstaVez = true;  // Alternating flag for left/right sensor reading

     // Center: always measure for maximum front update frequency
     unsigned long ahoraMs = millis();
#if USE_NEWPING
     if (echoListo[SEN_CEN]) {  // Check if center echo is ready
         noInterrupts();        // Disable interrupts for atomic operation
         unsigned int us = echoUs[SEN_CEN];  // Get echo timing
         echoListo[SEN_CEN] = false;         // Mark echo as processed
         interrupts();          // Re-enable interrupts
         float dist = (us == 0) ? 999.9 : (float)us / US_ROUNDTRIP_CM; // NewPing factor cm
         if (dist < DISTANCIA_MIN_VALIDA || dist > DISTANCIA_MAX_VALIDA) dist = 999.9;  // Validate distance
         distanciaUltrasonicoCentro_dbg = dist;  // Store debug value
         lecturasCentro[indiceLecturaCentro] = dist;  // Add to filter buffer
         indiceLecturaCentro = (indiceLecturaCentro + 1) % FILTER_SIZE;  // Advance buffer index
     }
#else
     if (ahoraMs - ultimoPingCenMs >= MIN_INTERVALO_PING_MS) {  // Check minimum interval
         float lecturaRawCen = medirDistancia(TRIG_CENTRO, ECHO_CENTRO);  // Measure center distance
         distanciaUltrasonicoCentro_dbg = lecturaRawCen;                   // Store debug value
         lecturasCentro[indiceLecturaCentro] = lecturaRawCen;              // Add to filter buffer
         indiceLecturaCentro = (indiceLecturaCentro + 1) % FILTER_SIZE;   // Advance buffer index
         ultimoPingCenMs = ahoraMs;                                        // Update last ping time
     }
#endif

     if (leerIzquierdaEstaVez) {  // Read left sensor this cycle
         
#if USE_NEWPING
         if (echoListo[SEN_IZQ]) {  // Check if left echo is ready
             noInterrupts();        // Disable interrupts for atomic operation
             unsigned int us = echoUs[SEN_IZQ];  // Get echo timing
             echoListo[SEN_IZQ] = false;         // Mark echo as processed
             interrupts();          // Re-enable interrupts
             float dist = (us == 0) ? 999.9 : (float)us / US_ROUNDTRIP_CM;  // Convert to distance
             if (dist < DISTANCIA_MIN_VALIDA || dist > DISTANCIA_MAX_VALIDA) dist = 999.9;  // Validate
             lecturasIzquierdo[indiceLecturaIzquierdo] = dist;  // Add to filter buffer
             indiceLecturaIzquierdo = (indiceLecturaIzquierdo + 1) % FILTER_SIZE;  // Advance index
         }
#else
         if (ahoraMs - ultimoPingIzqMs >= MIN_INTERVALO_PING_MS) {  // Check minimum interval
             float lecturaRawIzq = medirDistancia(TRIG_IZQUIERDO, ECHO_IZQUIERDO);  // Measure left distance
             lecturasIzquierdo[indiceLecturaIzquierdo] = lecturaRawIzq;              // Add to buffer
             indiceLecturaIzquierdo = (indiceLecturaIzquierdo + 1) % FILTER_SIZE;   // Advance index
             ultimoPingIzqMs = ahoraMs;                                             // Update ping time
         }
#endif
     } else {  // Read right sensor this cycle
         
#if USE_NEWPING
         if (echoListo[SEN_DER]) {  // Check if right echo is ready
             noInterrupts();        // Disable interrupts for atomic operation
             unsigned int us = echoUs[SEN_DER];  // Get echo timing
             echoListo[SEN_DER] = false;         // Mark echo as processed
             interrupts();          // Re-enable interrupts
             float dist = (us == 0) ? 999.9 : (float)us / US_ROUNDTRIP_CM;  // Convert to distance
             if (dist < DISTANCIA_MIN_VALIDA || dist > DISTANCIA_MAX_VALIDA) dist = 999.9;  // Validate
             lecturasDerecho[indiceLecturaDerecho] = dist;  // Add to filter buffer
             indiceLecturaDerecho = (indiceLecturaDerecho + 1) % FILTER_SIZE;  // Advance index
         }
#else
         if (ahoraMs - ultimoPingDerMs >= MIN_INTERVALO_PING_MS) {  // Check minimum interval
             float lecturaRawDer = medirDistancia(TRIG_DERECHA, ECHO_DERECHA);  // Measure right distance
             lecturasDerecho[indiceLecturaDerecho] = lecturaRawDer;              // Add to buffer
             indiceLecturaDerecho = (indiceLecturaDerecho + 1) % FILTER_SIZE;   // Advance index
             ultimoPingDerMs = ahoraMs;                                          // Update ping time
         }
#endif
     }
     leerIzquierdaEstaVez = !leerIzquierdaEstaVez;  // Toggle for next cycle

#if FILTER_SIZE == 1
     // Fast path: no median when window is 1
     distanciaIzquierdoFiltrada = lecturasIzquierdo[indiceLecturaIzquierdo == 0 ? 0 : (indiceLecturaIzquierdo - 1) % FILTER_SIZE];
     distanciaCentroFiltrada   = lecturasCentro[indiceLecturaCentro == 0 ? 0 : (indiceLecturaCentro - 1) % FILTER_SIZE];
     distanciaDerechoFiltrada  = lecturasDerecho[indiceLecturaDerecho == 0 ? 0 : (indiceLecturaDerecho - 1) % FILTER_SIZE];
#else
     distanciaIzquierdoFiltrada = calcularMediana(lecturasIzquierdo, FILTER_SIZE);  // Apply median filter
     distanciaCentroFiltrada = calcularMediana(lecturasCentro, FILTER_SIZE);
     distanciaDerechoFiltrada = calcularMediana(lecturasDerecho, FILTER_SIZE);
#endif
}

void actualizarControl() {
    unsigned long currentTime = micros();
    unsigned long elapsedTime = currentTime - prevTimePID;

    if (estadoActual == ESTABILIZANDO) {
        steeringServo.write(servoNeutral);  // Keep servo neutral during stabilization
        integral = 0.0f; derivative = 0.0f; prevError = 0.0f;  // Reset PID terms
        setpoint_actual_debug = setpoint;
        return;
    }

    if (elapsedTime <= 0) elapsedTime = sampleInterval * 1000UL;  // Prevent division by zero
    dt_global = elapsedTime / 1000000.0f;  // Convert to seconds
    prevTimePID = currentTime;             // Update PID timing reference
    
    bool congelarPidYaw = (estadoActual == PARKING && subEstadoParkingActual == SUB_PARKING_ALINEACION_LATERAL && intentandoAlineacionLateralActual);  // Freeze yaw PID during lateral parking
    
    float kp_actual, ki_actual, kd_actual;
    float setpoint_actual;
    switch(estadoActual) {
        case NORMAL: 
            kp_actual = Kp; ki_actual = Ki; kd_actual = Kd; setpoint_actual = setpoint;  // Normal navigation PID
            break;
        case EN_MANIOBRA: 
            kp_actual = Kp_turn; ki_actual = Ki_turn; kd_actual = Kd_turn; setpoint_actual = targetSetpoint;  // Turn maneuver PID
            break;
        case CORRIGIENDO:
            kp_actual = Kp_stab; ki_actual = Ki_stab; kd_actual = Kd_stab; setpoint_actual = setpoint;  // Stabilization PID
            break;
        case PARKING:
            if (subEstadoParkingActual == SUB_PARKING_ALINEACION_FRONTAL || 
                (subEstadoParkingActual == SUB_PARKING_ALINEACION_LATERAL && !intentandoAlineacionLateralActual)) {
                kp_actual = Kp; ki_actual = Ki; kd_actual = Kd;  // Normal PID for front alignment or lateral yaw control
                setpoint_actual = setpoint;
            } else {
                kp_actual = 0; ki_actual = 0; kd_actual = 0;  // Disable yaw PID during lateral positioning  (erica no tenemos los bloques) 
                setpoint_actual = currentYaw;
            }
            break;
        default: detenerMotor(); return;
    }
    
    setpoint_actual_debug = setpoint_actual;
    error = calcularErrorAngular(setpoint_actual, currentYaw);  // Calculate angular error
    
    if (fabs(error) <= deadband) { error = 0.0f; }  // Apply deadband to prevent oscillation
    
    float integralAntes = integral;  // Store integral for anti-windup
    if (!congelarPidYaw) {
        integral += error * dt_global;  // Update integral term
        integral = constrain(integral, -integralLimit, integralLimit);  // Apply integral windup protection
        derivative = (dt_global > 0.0001f) ? (error - prevError) / dt_global : 0.0f;  // Calculate derivative
        prevError = error;  // Store error for next derivative calculation
    }
    
    pidOutput = (kp_actual * error) + (ki_actual * integral) + (kd_actual * derivative);  // Calculate PID output
    
    // Unsaturated command (for anti-windup)
    int servoCommandSinSat = servoNeutral;
    if (estadoActual == PARKING && subEstadoParkingActual == SUB_PARKING_ALINEACION_LATERAL && intentandoAlineacionLateralActual) {
        servoCommandSinSat -= pdOutputParkLat;  // Use lateral parking PD control
    } else if (!congelarPidYaw) { 
        servoCommandSinSat -= pidOutput;  // Use normal yaw PID control
    }
    
    int servoCommand = constrain(servoCommandSinSat, servoMinAngle, servoMaxAngle);  // Apply servo limits

    // Anti-windup: if saturated under yaw control, revert integration of this cycle
    bool controlYawActivo = !(estadoActual == PARKING && subEstadoParkingActual == SUB_PARKING_ALINEACION_LATERAL && intentandoAlineacionLateralActual) && !congelarPidYaw;
    if (controlYawActivo && servoCommand != servoCommandSinSat) {
        integral = integralAntes;  // Revert integral to prevent windup
        pidOutput = (kp_actual * error) + (ki_actual * integral) + (kd_actual * derivative);  // Recalculate PID
        servoCommandSinSat = servoNeutral - pidOutput;  // Recalculate servo command
        servoCommand = constrain(servoCommandSinSat, servoMinAngle, servoMaxAngle);  // Apply limits again
    }

    // Servo speed limiter (slew rate)
    static int servoPrevCommand = servoNeutral;
    const float servoMaxDegPerSec = 220.0f;  // Maximum servo speed in degrees per second
    float deltaMax = servoMaxDegPerSec * dt_global;  // Maximum change per update
    if (deltaMax < 1.0f) deltaMax = 1.0f;  // Minimum change of 1 degree
    int delta = servoCommand - servoPrevCommand;  // Calculate desired change
    if (delta > (int)deltaMax) {
        servoCommand = servoPrevCommand + (int)deltaMax;  // Limit positive change
    } else if (delta < -(int)deltaMax) {
        servoCommand = servoPrevCommand - (int)deltaMax;  // Limit negative change
    }

    steeringServo.write(servoCommand);  // Send command to servo
    servoPrevCommand = servoCommand;     // Store for next slew rate calculation
}

void manejarEstados() { 
    EstadoRobot estadoAnterior = estadoActual;  // Store previous state for change detection
    float umbralLat, umbralCen;                 // Dynamic thresholds for lateral and central detection
    bool aperturaIzquierda, aperturaDerecha, paredCercana;  // Detection flags
    unsigned long tiempoTranscurrido;           // Elapsed time variable
    switch (estadoActual) {
        case NORMAL: {
             moverAdelante(velocidadNormal);  // Set normal navigation speed
             
             if (distanciaCentralInicial < 999.0) {
                  umbralLat = distanciaCentralInicial * umbralRatioLateral;  // Dynamic lateral threshold
                  umbralCen = distanciaCentralInicial / umbralRatioCentral;  // Dynamic central threshold
             } else { umbralLat = 30.0; umbralCen = 25.0; }  // Default thresholds if no reference

             aperturaIzquierda = (distanciaIzquierdoFiltrada < 999.0 && distanciaIzquierdoFiltrada > umbralLat);  // Left opening detected
             aperturaDerecha   = (distanciaDerechoFiltrada < 999.0 && distanciaDerechoFiltrada > umbralLat);    // Right opening detected
             paredCercana      = (distanciaUltrasonicoCentro_dbg < 999.0 && distanciaUltrasonicoCentro_dbg < umbralCen);  // Wall ahead
             
             if (paredCercana) {  // Wall detected ahead
                  DireccionGiro direccionDetectadaActual = NINGUNA;
                  
                  // Condition 2: determine if there is a valid opening.
                  if (aperturaIzquierda && (!aperturaDerecha || distanciaIzquierdoFiltrada > distanciaDerechoFiltrada)) {
                      direccionDetectadaActual = IZQUIERDA;  // Prefer left if better or only option
                  } else if (aperturaDerecha) {
                      direccionDetectadaActual = DERECHA;    // Turn right if left not available
                  }


                  if (direccionDetectadaActual != NINGUNA) {
                       if (direccionPrimerGiro == NINGUNA) {
                            direccionPrimerGiro = direccionDetectadaActual; // Set direction on first turn   xd
                       }
                       
                       // Only turn if detected opening matches the first turn direction.
                       if (direccionPrimerGiro == direccionDetectadaActual) {
                            targetSetpoint = setpoint + (direccionPrimerGiro == IZQUIERDA ? -90.0f : 90.0f);  // Calculate turn target
                            
                            while (targetSetpoint <= -180.0f) targetSetpoint += 360.0f;  // Normalize angle
                            while (targetSetpoint > 180.0f) targetSetpoint -= 360.0f;
                            
                            integral = 0.0f; derivative = 0.0f;  // Reset PID terms for new maneuver
                            prevError = calcularErrorAngular(targetSetpoint, currentYaw);  // Initialize error
                            tiempoInicioManiobra = millis();     // Start maneuver timer
                            contadorCurvas++;                    // Increment turn counter
                            estadoActual = EN_MANIOBRA;          // Change to maneuver state
                       }
                  }
             }
             break;
        }

        case EN_MANIOBRA: {
             moverAdelante(velocidadGiro);  // Set turning speed
             tiempoTranscurrido = millis() - tiempoInicioManiobra;
             bool anguloAlcanzado = (fabs(error) < TURN_ANGLE_TOLERANCE);  // Check if turn angle reached
             bool tiempoManiobraTimeout = (tiempoTranscurrido >= TIMEOUT_MANIOBRA);  // Check timeout
             if (anguloAlcanzado || tiempoManiobraTimeout) {
                  setpoint = targetSetpoint;  // Update current setpoint
                  integral = 0.0f; 
                  derivative = 0.0f; 
                  prevError = 0.0f; 
                  estabilizacionInicio = millis();  // Start stabilization timer   xd
                  estadoActual = ESTABILIZANDO;     // Change to stabilization state
             }
             break;
        }

        case ESTABILIZANDO: {
             moverAdelante(velocidadEstabilizacion);  // Set stabilization speed
             bool tiempoEstabilizacionCumplido = (millis() - estabilizacionInicio >= tiempoEstabilizacion);  // Check stabilization time
             if (tiempoEstabilizacionCumplido) {
                  integral = 0.0f; derivative = 0.0f;  // Reset PID terms
                  prevError = calcularErrorAngular(setpoint, currentYaw);  // Initialize error for correction
                  correccionInicio = millis();          // Start correction timer
                  
                  estadoActual = CORRIGIENDO;          // Change to correction state
             }
             break;
        }

       case CORRIGIENDO: {
            int velocidadActualCorreccion;
            tiempoTranscurrido = millis() - correccionInicio;
            velocidadActualCorreccion = map(tiempoTranscurrido, 0, tiempoCorreccion, velocidadCorreccionMin, velocidadCorreccionMax);  // Ramp speed during correction
            moverAdelante(constrain(velocidadActualCorreccion, velocidadCorreccionMin, velocidadCorreccionMax));  // Apply ramped speed

            bool correccionEstableYaw = (fabs(error) < CORRECTION_ANGLE_TOLERANCE);  // Check yaw stability
            bool tiempoCorreccionCumplido = (tiempoTranscurrido >= tiempoCorreccion);  // Check correction time x
            bool timeoutCorreccionTotal = (tiempoTranscurrido >= TIMEOUT_CORRECCION);  // Check total timeout   d

            if ((correccionEstableYaw && tiempoCorreccionCumplido) || timeoutCorreccionTotal) {
                integral = 0.0f; derivative = 0.0f; prevError = error;  // Reset PID terms xd
                
                if (contadorCurvas >= NUM_CURVAS_ANTES_DE_PARAR) {
                    estadoActual = PARKING;                     // Start parking sequence
                    subEstadoParkingActual = SUB_PARKING_NO_INICIADO;
                    detenerMotor();                            // Stop motor for parking
                    steeringServo.write(servoNeutral);         // Center servo
                } else {
                    estadoActual = NORMAL;                     // Return to normal navigation
                }
            }
            break;
        }

        case PARKING: {
            float distActualParedObjetivo;  // Current distance to target wall
            float distObjetivoLateral;      // Target lateral distance

            if (subEstadoParkingActual == SUB_PARKING_NO_INICIADO) {
                subEstadoParkingActual = SUB_PARKING_ALINEACION_LATERAL;  // Start with lateral alignment
                intentandoAlineacionLateralActual = true;
                tiempoInicioIntentoActual = millis();
                tiempoInicioSubEstadoParking = millis();
                paredObjetivoParkingEsIzquierda = (direccionPrimerGiro == DERECHA);  // Determine target wall side
                errorParkLat = 0.0; prevErrorParkLat = 0.0; pdOutputParkLat = 0.0;  // Reset lateral parking PID
            }

            if (millis() - tiempoInicioSubEstadoParking > TIMEOUT_PARKING_SUBFASE) {
                subEstadoParkingActual = SUB_PARKING_ALINEACION_FRONTAL;  // Move to front alignment after timeout
                tiempoInicioSubEstadoParking = millis();
                detenerMotor(); 
                pdOutputParkLat = 0.0; 
                integral = 0.0f; 
            } else if (subEstadoParkingActual == SUB_PARKING_ALINEACION_LATERAL) { 
                if (millis() - tiempoInicioIntentoActual > intervalo) {
                    intentandoAlineacionLateralActual = !intentandoAlineacionLateralActual;  // Alternate between lateral and yaw control
                    tiempoInicioIntentoActual = millis();
                    detenerMotor();
                    if (intentandoAlineacionLateralActual) {
                        prevErrorParkLat = 0.0f;  // Reset lateral error for new attempt xd
                    } else {
                        integral = 0.0f;  // Reset yaw PID integral
                        prevError = calcularErrorAngular(setpoint, currentYaw);  // Initialize yaw error
                    }
                }

                if (intentandoAlineacionLateralActual) {
                    moverAdelante(VELOCIDAD_PARKING_ALINEACION_LATERAL);  // Move forward during lateral alignment
                    
                    if (paredObjetivoParkingEsIzquierda) {
                        distActualParedObjetivo = distanciaIzquierdoFiltrada;  // Use left sensor
                    } else {
                        distActualParedObjetivo = distanciaDerechoFiltrada;   // Use right sensor
                    }
                    if (direccionPrimerGiro == DERECHA) {
                        distObjetivoLateral = distanciaIzquierdoInicial;  // Target left wall distance
                    } else {
                        distObjetivoLateral = distanciaDerechoInicial;    // Target right wall distance
                    }

                    if (distActualParedObjetivo >= 999.0) {
                        intentandoAlineacionLateralActual = false;  // Switch to yaw control if no wall detected
                        tiempoInicioIntentoActual = millis();
                        integral = 0.0f;
                        prevError = calcularErrorAngular(setpoint, currentYaw);
                        detenerMotor();
                    } else {
                        errorParkLat = distObjetivoLateral - distActualParedObjetivo;  // Calculate lateral error
                        if (fabs(errorParkLat) < DEADBAND_PARKING_LATERAL) {
                            errorParkLat = 0.0f;  // Apply deadband
                        }
                        float derivParkLat = (dt_global > 0.0001f) ? (errorParkLat - prevErrorParkLat) / dt_global : 0.0f;  // Calculate derivative
                        pdOutputParkLat = (Kp_parking_lateral * errorParkLat) + (Kd_parking_lateral * derivParkLat);  // Calculate PD output
                        prevErrorParkLat = errorParkLat;
                        if (!paredObjetivoParkingEsIzquierda) { 
                            pdOutputParkLat *= -1.0f;  // Invert control for right wall xd
                        }

                        if (fabs(errorParkLat) < 2.0) {
                            subEstadoParkingActual = SUB_PARKING_ALINEACION_FRONTAL;  // Move to front alignment when close enough
                            tiempoInicioSubEstadoParking = millis(); 
                            detenerMotor(); 
                            pdOutputParkLat = 0.0; 
                            integral = 0.0f; 
                        }
                    }
                } else {
                    moverAdelante(VELOCIDAD_PARKING_ALINEACION_LATERAL);  // Continue moving during yaw control phase
                }
            } else if (subEstadoParkingActual == SUB_PARKING_ALINEACION_FRONTAL) {
                if (distanciaCentroFiltrada >= 999.0) {
                    detenerMotor(); steeringServo.write(servoNeutral);  // Stop if no front wall detected
                    estadoActual = DETENIDO;
                    subEstadoParkingActual = SUB_PARKING_FINALIZADO;
                } else {
                  float errorFrontal = distanciaCentroFiltrada - distanciaCentralInicial;  // Calculate front distance error

                  if (fabs(errorFrontal) < DISTANCIA_CENTRAL_OBJETIVO_TOLERANCIA_PARKING || (millis() - tiempoInicioSubEstadoParking > TIMEOUT_PARKING_SUBFASE)) {
                       detenerMotor(); steeringServo.write(servoNeutral);  // Stop when close enough or timeout
                       estadoActual = DETENIDO;
                       subEstadoParkingActual = SUB_PARKING_FINALIZADO;
                       digitalWrite(stby, LOW);  // Disable motor driver ericka no tenemos los bloques
                  } else if (errorFrontal > 0) {
                      int servoCommandForward = servoNeutral - pidOutput;  // Use normal PID for forward movement  erica se robaron la pista
                      steeringServo.write(constrain(servoCommandForward, servoMinAngle, servoMaxAngle));
                      moverAdelante(VELOCIDAD_PARKING_AVANCE);
                  } else {
                      int servoCommandBackward = servoNeutral + pidOutput;  // Invert PID for backward movement
                      steeringServo.write(constrain(servoCommandBackward, servoMinAngle, servoMaxAngle));
                      if (direccionMotor == MOTOR_ADELANTE && velocidadActualMotor > 0) {
                          detenerMotor();  // Stop forward movement before reversing
                      } else {
                          moverAtras(VELOCIDAD_PARKING_RETROCESO);  // Move backward
                      }
                  }
                }
            }
            break;
        }

        case DETENIDO: {
             detenerMotor();  // Ensure motor is stopped xd
             break;
        }
    }

    if (estadoActual != estadoAnterior) {
        Serial.print(">>> State change: ");
        Serial.print(estadoNombres[estadoAnterior]);
        Serial.print(" -> ");
        Serial.println(estadoNombres[estadoActual]);
    }
}