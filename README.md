# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## Consideration
### The Model
* Student describes their model in detail. This includes the state, actuators and update equations.

The models used in this project are as follows.
Includes vehicle coordinates [x,y, psi], speed, and cross track error plus psi error (epsi). The output of the actuator is a(acceleration) and delta (steering angle).
"Delta" (from -0.436332 to 0.436332 in radians) meaning that the steering angle takes a value between -25 (deg) and 25 (deg). "a" is the acceleration, from -1.0 to 1.0.

* x: The x position of the vehicle.
* y: The y position of the vehicle.
* psi: The orientation of the vehicle.
* v: The current velocity.
* cte: The Cross-Track-Error
* epsi: The orientation error

The model combines the state and action from the previous time step and calculates the state of the current time step based on the following equation:

x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt  
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt  
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt  
v_[t+1] = v[t] + a[t] * dt  
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt  
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt  


### Timestep Length and Elapsed Duration (N & dt)
* Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

Time step length and frequency In this project, the waiting time is set to 100[ms]. Therefore, in order to properly control the vehicle, it must be estimated at least 100[ms] ago. Therefore, dt was set to 100[ms].
Latency is 100 [ms], but decreasing dt increases the resolution, but N needs to be lengthened accordingly. If N is increased, in that case, processing time is required. When dt is increased, the deviation of the prediction result becomes large. Also, when estimating a vehicle, it is not necessary to estimate for a long time, and 1000[ms] (100[ms] x 10) was appropriate.
Considering the case where the estimation period is set to 1000 [ms] by combination of other values. However, stable results could not be confirmed.
For exsample, (dt=0.25,N=4)/(dt=0.05, N=20), and so on.


### Polynomial Fitting and MPC Preprocessing
* If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

Waypoints are approximated by polynomials. Waypoints are converted from global coordinate system to vehicle coordinate system as preprocessing as follows. At each time step, the car is always at (0,0), so the calculation of the polynomial approximation is simplified.

double shift_x = ptsx[i]-px;  
double shift_y = ptsy[i]-py;  
ptsx[i] = (shift_x * cos(0-psi) - shift_y * sin(0-psi));  
ptsy[i] = (shift_x * sin(0-psi) + shift_y * cos(0-psi));  


### Model Predictive Control with Latency
* The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

Approaches to dealing with latency are largely the following two.
Change the equation of motion to one that takes latency into account as follows.

// Predict the state after 100[ms]  
double pred_x = v * delay_t;  
double pred_y = 0;  
double pred_psi = - v * steer_value * delay_t / Lf;  
double pred_v = v + throttle_value * delay_t;  
double pred_cte = cte + v * sin(epsi) * delay_t;  
double pred_epsi = epsi - v * steer_value /Lf * delay_t;  

Furthermore, it is necessary to set a value considering latency for throttle and steering as follows.

// actuation at time t-1 for latency consideration  
if(t > 0){  
	a0 = vars[a_start + t - 1];  
	delta0 = vars[delta_start + t - 1];  
}  

In addition, the following cost function was adopted in order to adapt the speed change in the curve. By adopting this function, it is possible to smoothly decelerate at high speed sharp curve and to prevent deviation.

// Increase in rapid curve at high speed  
//https://discussions.udacity.com/t/mpc-cost-paramter-tuning-question/354670/5  
fg[0] += 500 * CppAD::pow(vars[delta_start+t]*vars[v_start+t],2);


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
