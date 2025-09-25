# MERMAID
You can see all our flowcharts at detail in mermaid, in real time by [clicking here](https://www.mermaidchart.com/d/69ef36af-1adb-49cb-8dbc-4f43ecc426d1)😁

## First round simplified
<div align="center">
  <img width="500" height="700" alt="Documento A4 diagrama de flujo con flechas creativo doodle colorido (1)" src="https://github.com/user-attachments/assets/fc1641a8-fb5f-47e8-9ad7-3a566bcbc08c" />
</div>
<img width="1414" height="2000" alt="Documento A4 diagrama de flujo con flechas creativo doodle colorido (2)" src="https://github.com/user-attachments/assets/ffac0156-c2a8-4c57-ab14-c36262fa5657" />
<img width="1414" height="2000" alt="Documento A4 diagrama de flujo con flechas creativo doodle colorido (3)" src="https://github.com/user-attachments/assets/935af60f-f459-4545-87a7-3e781417b259" />
<div align="center">
  <img width="400" height="600" alt="Documento A4 diagrama de flujo con flechas creativo doodle colorido (4)" src="https://github.com/user-attachments/assets/8d5b8064-16a3-4a26-a8f0-fe534d25a86a" />
</div>

### Explanation

- Initialization: The robot starts, initializes all its hardware, and performs a crucial gyroscope calibration. If calibration fails, it stops. Otherwise, it waits for the start signal.
- NORMAL State: This is the default state. The robot drives forward while scanning with its ultrasonic sensors.
- Turn Detection: When a wall is detected ahead, it checks for a valid opening to the side. Critically, it will only commit to a turn if the opening's direction (left or right) matches the direction of the very first turn it ever made. This ensures it follows a consistent path (e.g., always taking left turns).
- Maneuver Sequence: A successful turn decision triggers a sequence of states:
  
 `EN_MANIOBRA`: Executes the 90-degree turn.
 `ESTABILIZANDO`: Drives straight for a moment to stabilize after the turn.
 `CORRIGIENDO`: Fine-tunes its heading to ensure it's perfectly aligned with the new path.
 
- Parking Logic: After completing the correction, it checks if it has made 12 or more turns. If not, it returns to the NORMAL state to find the next wall. If it has, it enters the PARKING state to perform its final alignment maneuver.
- End: Once parking is complete, it enters the DETENIDO (Stopped) state and the run ends.

<img width="961" height="3840" alt="Untitled diagram _ Mermaid Chart-2025-09-25-180711" src="https://github.com/user-attachments/assets/109994c1-eca9-429b-905c-fff3c7dad6d3" />


## BNO086

<img width="500" height="700" alt="BNO086 Flowchart" src="https://github.com/user-attachments/assets/a19538a9-7158-408c-a47f-694b4c5b92e6" />

- The `flowchart` begins with the initialization of the `BNO08X gyroscope`. Once powered on, the system verifies whether the sensor is detected; if the device is not found, the program immediately halts. If detection is successful, the `gyroscope` undergoes a `calibration` procedure to align its readings with a stable reference. A successful calibration allows the program to enter the `NORMAL` operating state, while a failure triggers repeated attempts. If calibration fails after retries, the system stops to avoid `unreliable data`.

- When in `NORMAL state`, the program enters its main loop. Inside the loop, it continuously checks whether the `BNO08X` has been reset. If a reset is detected, the system attempts a recalibration process. A failed recalibration results in termination, but a successful one restores the workflow. If no reset occurs, the program continues by reading sensor events. Each event is validated; invalid readings are discarded, and the loop resumes.

- Valid sensor events lead to the computation of the yaw angle, which represents the orientation of the system. This yaw value is then fed into a `PID controller` to maintain stability and control over the system’s heading. Once the `PID control` is applied, the loop cycles back, ensuring continuous monitoring, `correction`, and error handling throughout the system’s operation.

# End Page
Seeteki 2025 - Robot analysis and logic - flowcharts - first round
