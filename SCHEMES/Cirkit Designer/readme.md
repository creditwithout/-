## Cirkit Designer Main Circuit 😎
![Copy of Add a subheading (16 x 5 cm) (4096 x 2160 px) (2)](https://github.com/user-attachments/assets/56c7398b-dbeb-4fde-b94c-85732cc54c6d)
Circuit Designer is a program to build and simulate electronic circuits on your computer. It lets you place resistors, power sources, LEDs, and more like a digital breadboard. You can see how current flows and measure voltages without burning anything. It’s handy for learning, testing ideas, and avoiding mistakes before soldering. Basically, it’s your electrical lab on a screen.

We also decided to use this platform because it allows other users to view the circuits in real time, making it a much more interactive and organized experience. It should be noted that everything is public.

<img width="500" height="500" alt="image" src="https://github.com/user-attachments/assets/b82160e5-9b86-4f25-8c36-7c88dc9039c0" />

You can also visualize our circuit in SVG Format [here](https://app.cirkitdesigner.com/project/14ea10bb-b8b8-4b1f-92cd-de119d128b88)

## And by cliking [here](https://app.cirkitdesigner.com/project/14ea10bb-b8b8-4b1f-92cd-de119d128b88) you can get an interactive experience with our circuit

# Circuit EXPLANATION

- The central control unit of the circuit is an ```Arduino Nano ESP32```, which coordinates sensor inputs, actuator outputs, and communication interfaces.
- Three HC-SR04  ```ultrasonic sensors ``` are used for distance measurement at different positions: left, center, and right. Each  ```HC-SR04 ``` has four pins: Vcc (5V), GND, Trig (Trigger), and Echo.
- Vcc pins of all ultrasonic sensors are connected to the 5V power line from the Arduino.
- A servo motor (INJORA 7KG) is connected for mechanical movement, likely for rotating sensors or performing physical actions.
- A BNO086 (IMU) accelerometer/gyroscope/magnetometre module connects via the I2C protocol (SDA, SCL).
> [!IMPORTANT]
> Regarding the gyroscope that is in the circuit on GitHub, we no longer use that one; we now use the `BNO086`. However, it is from the original `SparkFun chip`, so we did not have time to modify it in the circuits, but all the original photos will show it. Nevertheless, it has the same connections as the previous one and better functionality, but it is basically the same, only with better electronics. You can find out more about the gyroscope [Here](https://www.sparkfun.com/sparkfun-vr-imu-breakout-bno086-qwiic.html), other wise we'll explain briefly here😁.

- The `SparkFun VR IMU` Breakout is a cutting-edge triple-axis accelerometer/gyro/magnetometer in a single package that you can connect via `I2C` and Qwiic.
  <img width="700" height="700" alt="image" src="https://github.com/user-attachments/assets/c762b33d-0135-41db-96cb-42dd0297f37b" />
