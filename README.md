# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

A video of the final simulation result can be found [here](https://github.com/uniquetrij/CarND-T2-P5-MPC/blob/master/result.mp4)

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


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

## Model Predictive Control (MPC)

Model Predictive Control (MPC) is a technoligy for advanced process control. MPC models can represent complex behavioral dynamics of a system. For the purpose of autonomous driving, if the problem of following a desired trajectory could be formulated as an optimization problem w.r.t the actuator values, we can then use an MPC to control the vehicle autonomously.

## The Model

### Preprocessing

The simulator provides the following data as an input to the program:
1. `ptsx` & `ptsy`, list of x and y coordinates of waypoints ahead of the vehicle
2. `x` & `y`, x and y coordinates of current position of the vehicle
3. `psi`, orientation of the vehicle (radians)
4. `speed`, current speed of the vehicle
5. `steering_angle`, current steering angle of the vehicle (radians)
6. `throttle`, current throttle value

The coordinates sent by the simulator are global coordinates of the map used by the simulator. These must be transformed to the vehicle's coordinates for convenience of computation of the error costs. This is done as follows:

```c++
for (int i = 0; i < ptsx.size(); i++) {
  double x_ = ptsx[i] - x;
  double y_ = ptsy[i] - y;
  ptsx_transformed[i] = x_ * cos(-psi) - y_ * sin(-psi);
  ptsy_transformed[i] = x_ * sin(-psi) + y_ * cos(-psi);
}
```
This essentially puts the vehicle at the origin with its face to the right horizontally.

Once the waypoints are transformed, the `polyfit()` function is used to obtain the coefficients of a polynomial curve of degree 3 that best fits the waypoints. Ideally, this is the trajectory that the vehicle should follow to stay on the track. Now the cross track error is simply the derivative of the polynomial and the psi error is the negative arc tangent of the derivative of the polynomial. But, since by the vehicle's coordinates, the vehicle is at the origin, the derivative of the curve is simply the second coefficient, i.e. coefficient of the term of order 1.

### Dealing With Latency 

A 100 ms latency is introduced in the program. The following equations describe the predicted latency state:

```c++
// latency until actuation
const double latency = 0.100;

// latency state
double latency_px = speed * latency; // cos(x) = 1 for x = 0
const double latency_py = 0.0; // sin(y) = 0 for y = 0
double latency_psi = speed * -steering_angle / Lf * latency; // psi = 0
double latency_v = speed + throttle * latency;
double latency_cte = cte + speed * sin(epsi) * latency;
double latency_epsi = epsi + speed * -steering_angle / Lf * latency;

```

These values are then passed on to the model for solving using the `MPC::Solve()` method.

### Designing The Error Cost Function

The goal is to make the vehicle follow the ideal trajectory by reducing the cross-track-error (cte) and the psi error (epsi) without causing severe constraint on speed and preventing sudden changes in steering; it must be a fast as well as a smooth ride. The parameters and weights mentioned below are the final weights adopted after a rigorous manual tuning.

Cost based on the current reference state includes the cte and epsi, both squared to eliminate (-)ve sign. The weights assigned to each are 5500 and 7500 respectively. It is also important to penalize for not maintaining some adequate velocity until the destination is reached. I incorporated the speed error as follows:

```c++
// costs pertaining to not maintaining the velocities.
// smaller values prevent the velocity from falling too low or 
// rising drastically high.
// larger values prevent velocity to always stay low.
fg[0] += 0.4 * CppAD::pow(vars[v_start + t] - 150, 2);
fg[0] += 0.8 * CppAD::pow(vars[v_start + t] - 120, 2);
fg[0] += 0.6 * CppAD::pow(vars[v_start + t] - 90, 2);
fg[0] += 0.8 * CppAD::pow(vars[v_start + t] - 60, 2);
fg[0] += 0.2 * CppAD::pow(vars[v_start + t] - 30, 2);
```
What that above code does is, it penalizes for speed being too low, as well as speed being too high. So the speed settles down somewhere in between. Otherwise if only a single reference velocity is chosen, the vehicle's speed becomes too erratic as the speed overshoots the reference velocity, comes down due to the error, goes up again, overshoots, and the cycle continues. This also prevents the speed from rising drastically high.

To avoid too frequent use of actuators or otherwise, I added squared errors for steering angle and acceleration with an weight of 5 each. It must also be noted that a large steering angle should be accompanied by a lower velocity. Similar argument holds for cte and epsi. Hence I added the following additional costs:

```c++
// additional costs for having high velocity
// when cte is large
fg[0] += 0.005 * CppAD::pow(vars[cte_start + t] * vars[v_start + t], 2);
// when psi error is large
fg[0] += 0.5 * CppAD::pow(vars[epsi_start + t] * vars[v_start + t], 2);
// when steering angle is large
fg[0] += 50 * CppAD::pow(vars[delta_start + t] * vars[v_start + t], 2);

```

Finally, to prevent sudden changes, I included squared errors for the change of the actuator values i.e. for steering and acceleration with weights of 100 and 10 respectively.

The `MPC::Solve()` method is responsible for optimizing the actuator values. The constraints of actuations are defined here as follows:

1. steering angle:  -0.436332 to 0.436332 radians (-25 to 25 degrees)

2. acceleration: -1.00 to 1.00

3. others (non actuators): -1.0e19 to 1.0e19 (max -ve to max +ve)

4. all non-initial state: 0

Finally, I used `ipopt::solve()` to obtain the optimized actuation values.

## Timestep Length and Elapsed Duration (N & dt)

Initially I felt that predicting for a longer duration will produce better results, but when I executed the program on the simulator, the results were otherwise. I selected various combinations on (N, dt) such as (25, 0.05), (20,0.1), (20, 0.05), (15, 0.1), (12,0.1), (5,0.05)and so on. What I realized that greater N is causing the model to slow down, while reducing dt was causing the model to preform worse. Eventually the optimal I could get was by using (N=10, dt=0.1) which essentially was a prediction for 1 second.

## Conclusion

The final model was able to drive the vehicle correctly in the simulator with speed reaching over 90 miles an hour. The ride was smooth and there were no sudden jerks or turns. The vehicle always stayed on track. [Here](https://github.com/uniquetrij/CarND-T2-P5-MPC/blob/master/result.mp4) is a video of the simulation results.








