
# Introduction
This project is the implementation of Extended Kalman Filter(EKF). The EKF here is used to estimate the position and velocity of bicycle around the Ego-car in the simulator.

# Definition
Kalman Filter in general is the most common estimation method. With kalman filter one can estimate the state of a dynamic system at any point in time. The estimation is  done by fusing the information taken from different sensors.


**In this project, we have two 'noisy' sensors:**
- A lidar sensor that measures our position in cartesian-coordinates `(x, y)`
- A radar sensor that measures our position and relative velocity (the velocity within line of sight) in polar coordinates `(rho, phi, drho)`

**We want to predict the position, and velocity  at any point in time:**
- In essence: the position and velocity of the system in cartesian coordinates: `(x, y, vx, vy)`
- NOTE: We are assuming a **constant velocity model (CV)** for this particular system

**This extended kalman filter does just that.** 


-----
# Code structure

The code consist of three main files

- FusionEKP:- This part(.h and .cpp) controls the code flow of the software. 
              Initialization of process and measurment covariance matrix , State   
              transition matrix etc has been done here.
              
- KalmanFilter:- This part is actual implementation of EKF. Both Peridiction and  
                 Update steps(for both of the sensors) has been implemented here. The 
                 Update of measurment covarince matrix(Q) and updating the Jacobaian 
                 matrix(Hj) has been done here. 
                 
- Tools      :- This part implements the helper function like calculating the RMSE 
                 values, Calculating the Jacobaian Matrix(Hj), converstion between 
                 polar to Cartesian co-ordinate system and vice versa
 
 
-----

# Result


The implementation has been done in the workspace. The initial values has been set for 
the different variables are as follow

 - measurement covariance matrix 
    R_laser = {0.0225, 0},
              {0, 0.0225}

 - measurement covariance matrix
    R_radar= {0.09, 0, 0},
             {0, 0.0009, 0},
             {0, 0, 0.09}

 - Measurement matrix 
    H_laser_  = {1, 0, 0, 0},
                {0, 1, 0, 0}
    

 - Initial state covariance
    P = [{1.0, 0.0, 0.0, 0.0},
             {0.0, 1.0, 0.0, 0.0},
             {0.0, 0.0, 1000.0,0.0},
             {0.0, 0.0, 0.0, 100}]
    

 - Initial state
     x ={1, 1, 1, 1}
     
     The code has been run against Dataset 1 and output RMSE value are less than the desired values. The output values are as follow:-
     
     px,py,vx,vy={0.0963,0.0854,0.3964,0.4505} 
    
    
    -----

# Conclusion

The goal of the project is to estimate the position and velocity of the bicycle moving around the Ego-car in the simulator. The EKF has been implemented in accordance with the points specified in the project rubric and the output values also full fill the criteria specified in the project rubric.