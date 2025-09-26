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
