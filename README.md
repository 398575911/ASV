### Code for ASV (Version 0.2.7)

We are developing the autonomous surface vehicle, including the environmental perception, behavioral planning, motion planning, path following, feedback control, etc. 
ASV will be a high performance, flexible architecture which accelerates the development, testing, and deployment of Autonomous Surface Vessels.

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version. 

Please let us know if you are using ASV, as we are curious to find out how it enables other people's work or research.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

#### Introduction

* C++ 17
* communication: socket TCP/IP, serial communication, checksum, etc
* controller: PID controller, thrust allocation, actuator, etc
* planner: Frenet Lattice generator, etc
* perception: Target tracking of Marine radar, etc
* math: library involving linear algebra, numerical analysis, etc
* fileIO: csv parser, Database (SqLite3), JSON, etc
* sensors: GPS, IMU, Wind, Marine Radar, etc
* 


#### TODO: 
1. Perception (e.g. Lidar, Marine Radar, camera, etc)
2. Hardware-in-loop simulation
3. Finite state machine
4. Message broker
5. export shared library


近期:
1. Hybrid A star
2. Marine radar visulization, target tracking
