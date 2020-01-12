# MissileTracker
MissileTracker is a MatLab control theory project that aims to model the movement of an intercepting missile as it targets another missile.

# Objvective
The objective is to move the mass to a target point at a target time.
This target point represents the target missile's position at the target time.
Reaching the target point at the target time indicates that the interceptor arrived at the same time as the target missile.
If this is the case, the interceptor and the target collide and the objective is met.

# Implementation
In the current model, the linear dynamics of a point mass are used to model the interceptor at a high level.
The basic control logic attempts to orient the missile so that it's pointing towards the target point, then adjust its speed so it arrives at the target time.
The high level logic uses 3 states, representing the forward velocity, angular position, and angular velocity of the missile.

This model sits on top of a lower level model, representing the missile's states in cartesian space.
The lower level model uses horizontal position, horizontal velocity, vertical position, and vertical velocity.
This representation allows for a much more sophisticated representation of the movement of the mass through two dimensional space.
An additional controller was designed to help the cartesian states of the system track the abstract states of the higher level controller.
Using this design strategy, high level control design can be done separately from the low level design and should still function as intended.

# Use
To run this code, use Matlab to run MissileTracker.m.
The dynamics for the high level angular model can be changed by modifying the AngularDynamics.m file.
The dynamics for the low level cartesian model can be changed by modifying the CartesianDynamics.m file.
Both files determine how their respective models respond to controller input.

The reference values each model tries to converge to are determined in MissileTracker.m.
This file can be modified to change how the values are calculated.

Currently, the target is simply a point and time determined by code in MissileTracker.m.
This file can be modified to simulate tracking different points or reaching points in different time frames.
