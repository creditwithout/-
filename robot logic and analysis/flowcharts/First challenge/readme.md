## First round simplified
<img width="500" height="700" alt="Documento A4 diagrama de flujo con flechas creativo doodle colorido (1)" src="https://github.com/user-attachments/assets/fc1641a8-fb5f-47e8-9ad7-3a566bcbc08c" />


## BNO086

<img width="1414" height="2000" alt="BNO086 Flowchart" src="https://github.com/user-attachments/assets/a19538a9-7158-408c-a47f-694b4c5b92e6" />

- The `flowchart` begins with the initialization of the `BNO08X gyroscope`. Once powered on, the system verifies whether the sensor is detected; if the device is not found, the program immediately halts. If detection is successful, the `gyroscope` undergoes a `calibration` procedure to align its readings with a stable reference. A successful calibration allows the program to enter the `NORMAL` operating state, while a failure triggers repeated attempts. If calibration fails after retries, the system stops to avoid `unreliable data`.

- When in `NORMAL state`, the program enters its main loop. Inside the loop, it continuously checks whether the `BNO08X` has been reset. If a reset is detected, the system attempts a recalibration process. A failed recalibration results in termination, but a successful one restores the workflow. If no reset occurs, the program continues by reading sensor events. Each event is validated; invalid readings are discarded, and the loop resumes.

- Valid sensor events lead to the computation of the yaw angle, which represents the orientation of the system. This yaw value is then fed into a `PID controller` to maintain stability and control over the system’s heading. Once the `PID control` is applied, the loop cycles back, ensuring continuous monitoring, `correction`, and error handling throughout the system’s operation.
