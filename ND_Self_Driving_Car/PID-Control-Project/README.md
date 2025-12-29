# # CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program


# Overview

This project implements a [PID controller](https://en.wikipedia.org/wiki/PID_controller) which control  a car in Udacity's simulator and prevent its from being off track. The simulator sends cross-track error, speed and angle to the PID controller(PID) and it receives the steering angle ([-1, 1] normalized) and the throttle to drive the car. 

# Rubric points

## Compilation

### Your code should compile.

The code has been developed in the project workspace. The code compiles without errors or warnings. No modifications were done on the provided setup.

## Implementation

### The PID procedure follows what was taught in the lessons.

The PID controller has implemented with the same concept as in the lessons. The method "UpdateError"calculates proportional, integral and derivative errors and the method "TotalError" calculates the total error using the appropriate coefficients.
## Reflection

### Describe the effect each of the P, I, D components had in your implementation.

- The proportional portion of the controller tries to steer the car toward the centre line (against the cross-track error). 

- The integral portion tries to eliminate a possible bias on the controlled system that could prevent the error to be eliminated. 

- The differential portion used to counter the trend of overshooting from the centre line by smoothing the approach to it.
- 
### Describe how the final hyperparameters were chosen.

The parameters were chosen manually by try and error. 
 
*  Setting the proportional value  the car starts  following the road but it starts overshooting . 
 
*  to overcome the overshooting factor I used the  differential to try to overcome the overshooting. The integral part only moved the car out of the road; so, it stayed as zero. 
 
*  After the car drove the track without going out of it, the parameters increased to minimize the average cross-track error on a single track lap. 

* The final parameters where [P: 1.5, I: 0.0, D: 2.5].

## Simulation

### The vehicle must successfully drive a lap around the track.

The vehicle has been driven around 15 minutes around the track with out any problem and the car has not been off the track during the test.
