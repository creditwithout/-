# 3.1 MODELS
In this section, we will explain each 3D part of our ARA 3.1 robot, which is the robot that will compete in the national WRO competition in Panama.

## Mount Tape
![Trabajo grupal de Los Sentidos Ilustativo Colorido (Mobile Video)](https://github.com/user-attachments/assets/741778fd-c738-4050-aa1f-0088cd825046)

Here you can find each part shown in the GIF, from the assembly of the robot's cover.

- [Main Upper mount](https://github.com/creditwithout/-/blob/main/3D%20Models/3.1%20(National)/ARAROBOT%20-%20Main%20tape%203.1.stl) : The main cover of the robot (everything is printed in Bambolabs ```PETG-CF``` filament, see below for more information) is mounted using M3 screws, and the entire top cover is assembled without screws for easier handling of the robot. It has a ```double circular tower system ``` so that it fits into each hole in the top plates. The bottom of the main plate has a horizontal support for the ``` LIPPO 12V battery```, so that it cannot rub against any components. It has a fairly minimalist design, with aggressive tips that save filament and lighten the weight of the robot.

- [Battery mount tape](https://github.com/creditwithout/-/blob/main/3D%20Models/3.1%20(National)/ARAROBOT%20-%20TAPA%20BATERIA%20(1).stl) : This is the top box that prevents the ```12V battery (LIPPO)``` from coming out, and it has holes that fit and lock into the main base to avoid using screws.
  
- [PCB Support](https://github.com/creditwithout/-/blob/main/3D%20Models/3.1%20(National)/ARAROBOT%20-%20PCB%20Mount%20tape.stl) : This is the base that supports our entire circuit and holds our printed circuit board (PCB) and some sensors and regulators.

> [!NOTE]
> Every ```3D``` piece is in the [3.1 (national)](https://github.com/creditwithout/-/tree/main/3D%20Models/3.1%20(National)).
<br>

## Steering
![Trabajo grupal de Los Sentidos Ilustativo Colorido (Mobile Video) (1)](https://github.com/user-attachments/assets/8a5bdfcb-1211-4c55-b6c5-cfe4f7ca26ee)

> [!NOTE]
> Upper are all the files that you can see in the Assembly GIF [3.1 (national)](https://github.com/creditwithout/-/tree/main/3D%20Models/3.1%20(National)).
<br>

We greatly improved the steering system for this national competition compared to previous regional competitions, since now that we do the first round in ```30 seconds```, we needed a greater angle of gro in case the ultrasonic sensor gave a delayed measurement, and thus avoid a collision. Therefore, we had to redesign the arms and servo fittings and the front arm amplitude so that the turning arc could have a much greater turning angle. This ```steering``` system no longer follows the guidelines of an ```Ackerman system```.

## Traction
![Trabajo grupal de Los Sentidos Ilustativo Colorido (Mobile Video) (2)](https://github.com/user-attachments/assets/e7bbac48-d8bb-4d6a-9eca-68e112bea7e0)

Most of the parts that make up the traction system are bolted together to provide greater stability and security in how each part is attached inside our robot.

<img width="363" height="245" alt="image" src="https://github.com/user-attachments/assets/1486af18-71fb-4976-abe7-6554bd926358" />

One of the many innovations we had was to add a ```differential``` to our robot, in order to eliminate skidding on curves and help achieve a wider turning angle. We designed this differential based on a crown gear set, which we will explain below with graphics, interactive illustrations, and some of the science behind it.


<img width="1979" height="780" alt="image" src="https://github.com/user-attachments/assets/16ec7807-1d64-4c7e-ac02-67be0bfa1009" />

- On the x-axis → time ```(𝑡)``` during a turning maneuver
- On the y-axis → angular velocity ```(𝜔)``` of each component in rad/s.

#### Three curves are plotted:
- (left wheel) → blue line: speed of the left wheel.
- (right wheel) → orange line: speed of the right wheel.
- (driveshaft) → dotted line: speed of the input shaft.
