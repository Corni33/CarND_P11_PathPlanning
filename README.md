# Path Planning

This project deals with path planning for a vehicle driving on a simulated highway. 
The vehicle should avoid collisions with other traffic participants, adapt its speed to preceding vehicles and change lanes if it's safe and reasonable.

## Path Generation

To drive the vehicle along the road, the simulator takes in a list of x-y-coordinates and moves the vehicle from one point to the next in 20 ms increments. 
The spacing of these points therefore affects the velocity, acceleration and jerk which the vehicle experiences. 
The vehilce's path must be planned in a "smooth" manner, to avoid exceeding comfortable or even physical limits of the vehicle.

In order to have the vehicle follow a continuous and smooth path, a new path is always planned under consideration of the previously planned path. 
The new path is generated as a spline which is tangentially connected to the end of the vehicles previous path. 
The other anchor points of this spline are calculated by moving further along the desired lane in 30 m increments (see line ... of main.cpp).
As this is done in a frenet frame defined along the middle of the road (reference path), the coordinates of the anchor points have to be transformed back into a cartesian coordinate system.
Because the waypoints defining the frenet frame are spaced relatively far apart, this transformation becomes choppy at the boundary between two linear pieces of the reference path.
I therefore implemented a function that uses spline interpolation between the waypoints, to always get a smooth transformation (lines ... of main.cpp). 

## Speed Control

The vehicle should ideally adapt its velocity like a real human driver. 
It should slow down when approaching another vehicle from behind and speed up when there is enough space and the desired target velocity is greater than the current velocity.
All this should be done considering the vehicle's limitations: Deceleration is limited by the friction between tire and road while the forward acceleration is limited by the motor.

I implemented the well known [Intelligent Driver Model](https://en.wikipedia.org/wiki/Intelligent_driver_model) (IDM) to generate acceleration values which respect the vehicle capabilites (line ... of main.cpp).
Using these acceleration values I calculated the spacing of the path points which will lead to exactly this acceleration being realized by the vehicle (line ...).
To achieve this, I also implemented a function to "walk along a spline" until some desired distance from another spline point has been reached (line ...).

## Lane Change

Changing lanes generally makes sense if it's both safe and reasonable. 
Both of these criteria are captured by the lane change model [MOBIL](http://traffic-simulation.de/MOBIL.html), which I implemented in my code (lines ...).
A lange change is performed if both the deceleration imposed on following vehicles is acceptable (--> safe) and a weighted gain in acceleration potential is ... (--> reasonable)


