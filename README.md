# Autonomous-Robotics-Systems

  The platform used is a Pololu-ROMI composed of a 165mm circular chassis, which is equip-
ped with two 70x8mm wheels, placed on the axis that goes through the center of the platform,
which endow the platform with differential locomotion characteristics. Each wheel has a qua-
drature incremental encoder based on Hall effect sensors. Using 6-pole magnetic disks, each
encoder provides a resolution of 12 pulses / turn of the motor, which corresponds to approxi-
mately 1440 pulses / turn of the wheel.

The algorithms were written in C++ and the compiler used was MBED. 

# TP1 - Movement Control for a differential mobile platform

• Integration into the Mini Explorer platform the functions necessary for platform control
and sensing;
• Development and implementation of locomotion kinematics of the mobile differential
platform;
• Implementation of open space locomotion control algorithms.
 
# TP2 - Virtual Force Fields and Histogram implementation for Mobile Robots Navigation

The whole point of this work was to take the robot from point A to point B with the
robot’s respective navigation. Whether it is VFF or VFH, the last being an updated version
of the other, it has to reach the point B with a smooth traject and avoid all the existent
obstacles in the map.
In the figure below, it is represented the robot’s universe, the histogram and the active region,
where he makes the measurements and the respective calculations. This region’s has the
following dimensions:

• Cell: 5x5cm

• Histogram: 400x400cm (80x80 Cells)

• Active region: 55x55cm (11x11 Cells)

![alt text](https://github.com/lcruz1618/Autonomous-Robotics-Systems/blob/master/histogram_active_region.png)

# TP3 - Mapping using Bayesian models and occupational grid

This algorithm detects objects or obstacles using a two dimensional laser through the RPLIDAR sensor.
This device allows real-time mapping of the navigation area of the robot. Until here this was being inserted manually
with the programmer's knowledge of the map. This new functionality improves the algorithm, making the robot autonomous and 
making possible a more dynamic navigation.


