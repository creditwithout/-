# Flowcharts Second Round
## General flowchart

<img width="3840" height="3165" alt="Second Round flowchart _ Mermaid Chart-2025-09-26-130019" src="https://github.com/user-attachments/assets/e22751d3-2e78-4f98-8adb-bee3c6ee9e54" />


> [!NOTE]
> If the picture is too tiny you can watch the flowchart [here](https://www.mermaidchart.com/d/1aa4808a-1b44-4b7c-8cd6-f1c42f7b56fd).
<br>

- This flowchart illustrates the high-level decision-making process of the robot throughout its entire mission, from startup to shutdown. It follows the main state machine defined in the code, which dictates the robot's behavior at any given moment.

- The process begins with a critical `initialization` and `calibration phase`. The robot activates its hardware, including `motors and sensors`, and performs a gyroscope `calibration` to establish a stable `zero-point` for navigation. A failure in this stage is critical and halts the program, ensuring the robot doesn't start with faulty sensors. Once calibration is successful, it waits for a physical start button press to begin the main operational loop, starting with the `SALIR_PARKING` (Exit Parking) maneuver.
The core of the logic resides in a continuous loop that repeatedly checks sensor data and executes actions based on the robot's current state. The primary decision point in each loop is the state router. It first handles high-priority events, like a sudden IMU reset or the detection of an obstacle `("blob")`, which immediately triggers the `ESQUIVAR_BLOQUE` (Evade Block) state.
If no priority events occur, the robot executes the logic corresponding to its current state. In the NORMAL state, it drives forward, looking for turn conditions. States like `EN_MANIOBRA (In Maneuver)` and `CORRECCION_REVERSA` (Reverse Correction) handle the complex sequence of turning and repositioning. The `ESPERANDO_ (Waiting)` states act as crucial synchronization points, pausing the robot to wait for confirmation from the camera system before proceeding. This structured state-based approach allows the robot to systematically navigate the course, handle turns, evade obstacles, and finally enter the `DETENIDO (Stopped)` state after completing its objectives.


## Gyroscopoe BNO086 Flowchart Sencond Round
<img width="1573" height="3840" alt="Untitled diagram _ Mermaid Chart-2025-09-26-155035" src="https://github.com/user-attachments/assets/5d8519c0-bd11-44dc-bd63-16015506262a" />



> [!NOTE]
> If the picture is too tiny you can watch the flowchart, (it's different from the one above) [here](https://www.mermaidchart.com/d/9df89e61-509a-440d-925c-24cbaa2acc8d).
<br>


- The process starts in the `setup phase`, where the robot first attempts to establish communication with the `IMU`. A failure here is fatal and stops the robot from proceeding. If successful, it enters a mandatory calibration routine. During calibration, the robot collects multiple orientation samples over a short period to calculate a `yawOffset`.

- This offset corrects for any initial sensor drift and establishes a reliable `"straight-ahead"` reference. This step is vital for accurate navigation, and the code includes timeouts and retry limits to ensure it completes successfully before the main loop begins.
In every single iteration of the main loop, the first and most critical check is `myIMU.wasReset()`. An `IMU` might reset due to power fluctuations, which would invalidate all orientation data. If a reset is detected, the robot immediately stops all movement to prevent crashing. It then attempts an on-the-fly recalibration. If successful, it carefully resets its navigation variables and resumes the mission from a safe state (`NORMAL` or by restarting the current maneuver). This robust error-handling logic is crucial for autonomous reliability.

- If no reset has occurred, the robot proceeds with normal operation. It polls the `IMU` for a new sensor event, specifically looking for the stable `"Game Rotation Vector."` From this, it reads quaternion data, which is then mathematically converted into a clear currentYaw angle representing the robot's heading. This final currentYaw value is the primary output of the gyroscope logic and is fed directly into the `PID control algorithm`. The [PID controller](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller) uses this value to calculate the necessary steering adjustments to keep the robot on its intended path, completing the sensor-to-action cycle before the loop repeats.

## Camera Code [Open MV H7 Plus](https://openmv.io/products/openmv-cam-h7-plus?gad_source=1&gad_campaignid=21060752326&gbraid=0AAAAADOSICqh367zSiwIAszfGRs-U7cLM&gclid=CjwKCAjw89jGBhB0EiwA2o1OnxK3NecyiK3j0OXD5AUHRwgyzc3UoXTe6ozS7JMpBEWSMGlq2gf2PBoC9SwQAvD_BwE) In the Arduino CODE

<img width="2015" height="3840" alt="Untitled diagram _ Mermaid Chart-2025-09-26-162527" src="https://github.com/user-attachments/assets/166b967c-dd9c-49e5-bf98-6678bb6414f6" />

1- `Initialization`:
At the start, the robot configures and activates the Serial1 port, which is the exclusive communication channel with the camera.

2- `Main Cycle` (Repeats Constantly):
Step 2.1: Listen: The robot checks if the camera has sent any data through the Serial1 port.
If no data: It starts the cycle over again.
If there is data: It proceeds to the next step.

- Step 2.2: Read: It reads a single character from the sent data.
- Step 2.3: Build the Message: It adds the character to a temporary memory (a "buffer"). It repeats this process, character by character, until it detects a "newline" character.
- Step 2.4: Message Complete: When it detects the "newline," it means a complete message has been received (e.g., "B,3,1"). Now, it proceeds to process it.
Message Processing
- Step 3.1: Identify Type: The robot looks at the first letter of the message to know what it means.
- 
3- `If the letter is 'P' (Wall)`:
This indicates the camera is answering a question about whether there is a wall.
It extracts the number that follows (an 8 or a 9).
It saves that number in a variable so the robot can decide whether it should turn.
If the letter is 'C' (Check):
This is a general response from the camera.
It extracts the number that follows (a 0 or a 1) to know if the camera sees lane markings.
It activates a flag (checkCompleto) to notify the robot that it can stop waiting and proceed with the next action.
If the letter is 'B' (Blob / Obstacle):
This means the camera has detected an obstacle.
It extracts the two numbers that follow (the section and the roi).
It saves these numbers in variables so the robot can decide if it should initiate an evasion maneuver.
4- `Finalize and Repeat`:
After processing the message, the robot clears the temporary memory and returns to Step 2.1 to wait for the next message from the camera
