# MissileTracker
MissileTracker is a MatLab control theory project that aims to model the movement of an intercepting missile as it targets another missile.

# Objvective
The objective is to move the mass to a target point at a target time.
This target point represents the target missile's position at the target time.
Reaching the target point at the target time indicates that the interceptor arrived at the same time as the target missile.
If this is the case, the interceptor and the target collide and the objective is met.

# Implementation
In the current model, the linear dynamics of a point mass are used to model the interceptor at a high level.
The controller uses state feedback to follow a trajectory leading to the target.
The trajectory is a straight line from the point to the target.
The controller also converges to a target velocity, calculated by dividing the distance to the target by the time remaining until collision.

The missile's actuator/thruster limits are modeled by projecting the control input onto the missile's relative (x,y) plane.
The forward and orthogonal vectors of motion for the missile are calculated and normalized, then the controller input is projected onto these two vectors.
The orthogonal limit to the orthogonal and forward forces the thrusters can apply is modeled by simply limiting the values along these axes.
The limited controller values are then applied to the model and used to calculate the real trajectory of the missile.

# Use
To run this code, use Matlab to run MissileTracker.m.
The dynamics for the model can be changed by modifying the CartesianDynamics.m file.

The reference values the model tries to converge to are determined in MissileTracker.m.
This file can be modified to change how the values are calculated.

Currently, the target is simply a point and time determined by code in MissileTracker.m.
This file can be modified to simulate tracking different points or reaching points in different time frames.
