# Localization with Particle Filters - Finding a Kidnapped Vehicle


## Project Overview
This project deals with localization of vehicle using a 2 dimensional Particle filter.
The vehicle is at some unknown location. 

In this project the map data is used to  initialize  the vehicle position. I created a 100 particles which were initialize as a gussian distribution. Then the next step is predicting the car position using the velocity and yaw. After that the next step is update in which first I transformed sensor observation points into my map coordinates, associated these observations with landmarks on the map, and then calculated the likelihood that a given particle made those observation based off of the landmark positions in the map. The particles were then re-sampled based on how likely a given particle was to have made the observations of the landmarks, which helps to more accurately localize the vehicle.
Along with sufficient accuracy to have localized the vehicle within a small amount of space, the project also required having an efficient algorithm, as there was a time limit to how long it could run for without failing the given parameters.


## Running the Code
I implemented the particle filter in project workspace using the following commands the code compile and run (with the simulator)

```
> ./clean.sh
> ./build.sh
> ./run.sh
```


## Results
### Performance
According the requirements the x and y vary in range of 1 meter(max_translation_error) and yaw should vary in range of 0.05(max_yaw_error), After testing my code with the simulator the results looks good and I got the message of successfully passing the requirements. 
