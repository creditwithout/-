
![Add a subheading (16 x 5 cm) (4096 x 2160 px) (4)](https://github.com/user-attachments/assets/72b55f8b-9091-4b58-a246-8826f7c33bf9)

# 1 Sensor Fusion Graph (Complementary Filter)
<img width="1400" height="800" alt="Generated Image September 26, 2025 - 8_24AM" src="https://github.com/user-attachments/assets/de6ad322-db04-4097-b434-882e1a346bd5" />

- Ths `graph` illustrates the fundamental process of sensor fusion, specifically using a complementary filter to obtain an accurate and stable angle estimation. It masterfully showcases the individual weaknesses of an accelerometer and a gyroscope and demonstrates how combining them yields a superior result.

- The thin, erratic green line represents the angle derived from the `accelerometer`. While this signal is incredibly noisy and unreliable for instantaneous measurements due to its high sensitivity to external vibrations, its long-term average is highly accurate. It does not drift because it uses the constant force of gravity as its reference, correctly centering around the true angle of zero.
  
- In contrast, the smooth, thin red line shows the angle from the gyroscope. This signal is excellent for tracking rapid, short-term changes in orientation without noise. However, it suffers from a critical flaw known as "drift," where small, inherent errors accumulate over time, causing the angle to slowly but steadily wander away from the true value.
  
The thick blue line is the final, fused output from the complementary filter. This algorithm intelligently combines the best attributes of both sensors. It primarily relies on the smooth, responsive data from the gyroscope for its moment-to-moment readings. Simultaneously, it uses the noisy but stable long-term data from the accelerometer as a constant, reliable anchor.
This process effectively corrects the gyroscope's drift by continuously "pulling" its smooth signal back towards the correct average provided by the accelerometer.

# 2  9-Axis vs. 6-Axis Sensor Fusion
![WhatsApp Image 2025-09-26 at 8 27 46 AM](https://github.com/user-attachments/assets/306d4c11-6e32-47e0-ab34-9debfb363ac7)

- The dashed black line represents the Real Yaw, the true, error-free orientation that both systems are attempting to replicate.
The red line shows the output of a standard 6-Axis Fusion, which combines only a gyroscope and an accelerometer. While this method is effective for calculating pitch and roll, it lacks an absolute reference for the yaw direction. Consequently, it is completely vulnerable to the gyroscope's inherent drift. As the graph clearly illustrates, this accumulated error causes the red line to continuously and progressively deviate from the real yaw, ending with a significant error of over 30 degrees.

- In stark contrast, the blue line represents the output of an advanced 9-Axis Fusion, such as that performed by a `BNO086 sensor`. This system adds a crucial third sensor: a magnetometer, which acts like a compass by providing an absolute reference to the Earth's magnetic field. This allows the fusion algorithm to constantly correct the gyroscope's drift, resulting in an exceptionally accurate yaw calculation that almost perfectly overlays the real yaw line.


# 3 Motor Acceleration Control (Velocity Ramp)
<img width="1000" height="600" alt="Generated Image September 26, 2025 - 8_08AM (1)" src="https://github.com/user-attachments/assets/e896548e-b0d2-492a-8add-6beeea507c53" />

- The dashed red line represents the "Instantaneous Start." In this approach, the software commands the motor's PWM value to jump from zero to its target speed in a single, infinitesimal step. This demand for an instantaneous change in velocity is physically impossible for any mechanical system with inertia to achieve. The attempt to follow this command results in a sudden surge of current and maximum torque, causing abrupt, jerky movement. This not only puts significant stress on the motor and drivetrain components but also frequently leads to a loss of traction, causing the wheels to spin in place.
  
- In stark contrast, the solid blue line demonstrates the superior "Ramped Start" method. Here, the motor's speed command is not sent all at once. Instead, the PWM value is increased gradually and linearly over a defined period—in this case, 0.5 seconds—until it reaches the desired target speed.
 
- This controlled and progressive application of power allows the robot's physical momentum to build up smoothly and naturally. It ensures that the torque delivered to the wheels is always manageable, thereby maximizing grip and preventing wheel spin. The result is a smooth, predictable, and efficient launch that protects the hardware from mechanical shock and provides far more reliable control over the robot's movements.

# 4 - Explanation of PID Controller Simulation for Heading Correction
<img width="1200" height="700" alt="Generated Image September 24, 2025 - 9_00PM (1)" src="https://github.com/user-attachments/assets/b6644634-17e4-4852-b234-5fa3efcad33c" />

- The dashed red line represents the target, or "setpoint," which is a heading of 0.0 degrees. The solid blue line shows the robot's actual heading over time. At the beginning of the simulation, the robot has a significant initial error, starting at a negative angle far from its goal.
The PID controller's job is to eliminate this error. The initial, steep rise of the blue curve is primarily driven by the `Proportional (P)` term, which applies a strong corrective force that is directly proportional to the large initial error.

- As the robot's heading gets closer to the target, the error decreases, and the corrective force lessens. The smooth, non-oscillating curve demonstrates the crucial role of the Derivative (D) term, which acts as a damper by anticipating the rate of change. It prevents the robot from overshooting the target and oscillating around it.
Finally, the `Integral (I)` term works to eliminate any small, residual steady-state error, ensuring the robot doesn't just get close to the target but settles precisely on it.

# End Page
Seteki 2025 - robot analysis and logic - Graphics
