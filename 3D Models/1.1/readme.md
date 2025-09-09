## EXPLANATION / Robot Chasis 🚗



In this part 2 we are going to explain everything that supports our robot, that is the base, how we made it, the reason for each part, the platforms we used to design everything related to the chassis, all the implements we used physically, and how each mechanical part of our cart or robot works.

### Platforms we use 
For the chassis of our cart in our team we decided to adjust the chassis based on our needs, and that function is fulfilled by 3D printing, as it allows us to make the parts just to the extent in which we require them, as a first point we use two platforms for 3D modeling, OnShape and `FUSION`, although most of the robot is made in Onshape.
<br>


Onshape is a cloud-based 3D CAD platform that allows real-time collaboration and requires no installation, as it runs entirely in a web browser. It supports cross-platform access from any device, making teamwork easy and efficient. Key advantages include built-in version control, secure cloud storage, and integrated tools for design and data management. With fast iteration, instant sharing, and automatic backups, Onshape streamlines modern product development.



Most of our robot is made on this platform, in which we already gave a little introduction about it, when we go breaking down each 3D part of the robot will be explained.

### FUSION 360: 

Fusion 360 is a cloud-based 3D CAD, CAM, and CAE software developed by Autodesk. It combines design, engineering, simulation, and manufacturing in one platform. Users can create precise 3D models, assemblies, and technical drawings. Its parametric and freeform modeling tools allow flexibility and fast changes. Fusion 360 also offers simulation features to test stress, motion, and heat. With built-in CAM, it supports CNC machining up to 5-axis. It includes PCB and electronics design, making it ideal for integrated projects. Cloud collaboration enables real-time teamwork and version control. Fusion 360 is user-friendly, cross-platform, and widely used by students, engineers, and makers.




the major use we made of FUSION 360 was to model the gears that we implemented in the robot, the use of FUSION was important for the team to model, and to implement `FUTURE IMPROVEMENTS` for our robot.

__________________________________________________________________________________

### Robot Main Chasis support

For the base of our robot, we decided to design it on the [OnShape](https://www.onshape.com/en/) platform, and the main points we wanted to cover to design our main chassis are those of: <br><br>
▶️ `Lightweight model`<br>
▶️ `Friendly and integrated model at the time of physical assembly.`<br>
▶️ `Futuristic but practical and simple model.`<br><br>

> [!NOTE]
> To see each 3D model of our robot on GitHub you can click [`HERE`](https://github.com/seteki/III-Wind/tree/main/3D-models), or go directly to the folder of `3D models` of our robot, or you can also see them graphically in the [`README.md`](https://github.com/seteki/III-Wind/blob/main/README.md) and below each image or `GIF` of our 3D pieces open a link to see the modeling shown, Thank you :)



For the base of our robot we did it on the basis of four homogeneous parts, which would be: <br>

▶️ `Ultrasonic Sensors`<br>
▶️ `Steering`<br>
▶️ `Motor and shaft`<br>
▶️ `Top plate`<br><br>


### Steering Parts

Our 3D `Steering` made in onshape, was made with the plans to make it in a simple but functional way, and it is mainly based on an `Ackerman system`, but as you can clearly see in our address, we only took the principle because in itself it has quite a few differences but in essence it is the base of an `Ackerman system`.



<div align="center">
  <img src="https://github.com/user-attachments/assets/2d60315c-ab8f-4316-b973-59154d61b092" alt="Texto descriptivo" width="600"/>
</div>



> [!IMPORTANT]
> To more details in the explication of `Ackerman System`, PLease go [Here](https://www.mathworks.com/help/vdynblks/ref/steeringsystem.html).

The Ackermann steering system is a geometric arrangement used on vehicles with two steerable front wheels, designed so that during a turn all four wheels roll along concentrc arcs without lateral slip. 
When you turn, the inner wheel must pivot more sharply than the outer one, because it follows a tighter circular path (radius = R – T/2) while the outer wheel follows a looser path (radius = R + T/2). Taking the arctangent of the wheelbase divided by each effective radius yields the exact mechanical angle each wheel must achieve so that all wheels roll without sideways slip. This precise relationship minimizes tire scrubbing, distributes cornering forces correctly, and ensures the vehicle tracks smoothly through turns.

▶️ `Mechanical Implementation`:
Steering linkage (“tie rod”) connects the two steering knuckles. A fixed pivot point on the chassis or steering arm drives the tie rod so that lateral movement produces the correct differential angles between inner and outer wheels.

▶️ `Ackermann Angle`:
The steering arms (on the knuckles) are canted such that imaginary lines drawn through them meet at a point on the rear axle.
The exact cant angle depends on track width and the chosen tie‑rod attachment points.


