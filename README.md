## Udacity - Self Driving Car Nanodegree (Term 2) - MPC Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Background
---
In this project, my goal is to build a Model Predictive Control that will autonomously drive a car around a track in a simulation.

Overview of Repository
---
This repository contains the following source files that I have forked from the [main repository](https://github.com/udacity/CarND-MPC-Project) and subsequently modified for the MPC project:

1.  [MPC.cpp](https://github.com/MartinKan/CarND-MPC-Project/blob/master/src/MPC.cpp)
2.  [MPC.h](https://github.com/MartinKan/CarND-MPC-Project/blob/master/src/MPC.h)
3.  [main.cpp](https://github.com/MartinKan/CarND-MPC-Project/blob/master/src/main.cpp)

Summary of the MPC model
---
The MPC class implements the MPC model using the IPOPT optimizer, which “solves" the problem of coming up with the best trajectory (and the actuator values to use at each timestamp to create that trajectory) for the car given a defined set of constraints, cost equations and kinematic model. This MPC model is summarized as follows:

![alt text](https://github.com/MartinKan/CarND-MPC-Project/blob/master/images/model_setup.JPG)

In this model, the duration of the predicted trajectory is defined by two variables: N and dt.  N is the number of timestamps in the trajectory and dt is the difference in time between each successive timestamps (in seconds).  The product of N and dt will give you the total duration of the predicted trajectory (which will be represented in the simulator as a green line).

For each timestamp along the predicted trajectory, the MPC model computes the predicted state of the vehicle, which includes its x and y coordinates (i.e. its location), angle, velocity, cross track error (distance from the center line) and angle deviation.  The predicted state of the vehicle at each timestamp is computed by feeding the following information into the IPOPT optimizer:

- previous state of the vehicle
- the kinematic model (as depicted in the diagram above)
- the actuator constraints (i.e. constraints of the steering angle and throttle)
- the cost equations

The IPOPT optimizer will solve the problem of finding the best trajectory for the vehicle given these criteria.  The optimizer will return a giant vector that contains the values of the predicted state at each timestamp as well as the actuator inputs that are required to achieve those predicted values.  The values of the actuator inputs can then be used by us to drive the vehicle in the simulation around the track.

One thing that is worth mentioning: the choice of f(x) (used in the kinematic model to calculate the CTE) is determined by us.  In the project, we were advised to use a third order polynomial for f(x) since it is able to mimick curved lines quite well.  The result is a predicted trajectory that behaves like a third order polynomial.

Selection of values for N and DT
---

I have experimented with different combinations of N and DT and settled in the end for N = 15 and dt = 0.04.  This produces a predicted trajectory of 0.6s in duration and it was both relatively easy for the optimizer to solve and provided sufficient data to the optimizer to manoeuvre the corners gracefully, as shown below:

![alt text](https://github.com/MartinKan/CarND-MPC-Project/blob/master/images/Tuned.gif)

When a higher value of N is chosen, the problem becomes harder to solve and it may occasionally result in the car going off track for this reason.  Below is an example of this when the values N = 20 and dt = 0.04 are chosen:

![alt text](https://github.com/MartinKan/CarND-MPC-Project/blob/master/images/N20DT004.gif)

Alternatively, when a smaller value of N is chosen, it may not provide sufficient data to the optimizer to manoeuvre the corners gracefully.  Below is an example of this when the values N = 10 and dt = 0.04 are chosen, you can see how the vehicle clipped the curb when it made the right turn:

![alt text](https://github.com/MartinKan/CarND-MPC-Project/blob/master/images/N10DT004.gif)

When a higher value of DT is chosen, it also made the vehicle veer off the tracks.  Presumably because it takes longer for the vehicle to self correct when errors occur, as shown below when values N = 15 and dt = 0.1 are chosen:

![alt text](https://github.com/MartinKan/CarND-MPC-Project/blob/master/images/N15DT01.gif)

Data preprocessing
---

To help compute some of the input data that feeds into the MPC model (e.g. CTE), I preprocessed the waypoints and transformed them from the map space to the car space using a rotation matrix each time prior to invoking the MPC code (lines 99 to 110 of main.cpp).  I then fit a polynomial to the waypoint coordinates (lines 112 to 115 of main.cpp) and proceed to compute the cross track error and the orientation error of the vehicle (lines 117 to 120 of main.cpp).  It is easier to transform the waypoints from map space to car space before working out the CTE and orientation error, since in car space, the X and Y coordinates and orientation of the car are all equivalent to 0.  Once we worked out the CTE and orientation error of the vehicle, we then load them into a state vector along with the vehicle's X and Y coordinates in car space (i.e. (0,0)), orientation in car space (also 0) and its velocity (returned to us by the simulator).  The actuator inputs are not loaded into the state vector as they are not needed by the MPC model.  This state vector is then passed to the MPC code to compute the best trajectory given the current state.

Dealing with latency
---

I deal with the 100ms latency in my code by using a separate set of kinematic equations that factor in the latency by substituting dt with the amount of the latency (lines 170 to 178 of MPC.cpp).  This will compute the new state space of the vehicle after the latency has elapsed.

Running the code
---

To run the code, first compile the code by running the following command:

	cmake .. && make

Then run the code by executing the following command:

	./mpc

Alternatively, the code can be ran by using command line arguments:

	./mpc [N] [dt] [v] [M1] [M2] [M3] [M4] [M5] [M6] [M7]

Where M1 - M7 represent the multipliers for the cost functions.
