# CarND-Controls-PID

Implementation of a PID controller in C++ to maneuver the vehicle around the track!

This project involves a Unity Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases). The simulator provides the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.

The hyper-parameters have been chosen based on manual tuning. If there is too much oscillation, either the proportional gain was reduced or the derivative gain was increased until it stabalizes. The integral term was set to zero.

Kp = 0.15; Ki = 0.0 ; Kd = 2.5

```
PID pid;
// Initialize the pid variable.
pid.Init(0.15, 0.0, 2.5);
```


##### Simulator Output Recording


[![PID Control](https://img.youtube.com/vi/cQ5XvudEdjE/0.jpg)](https://www.youtube.com/watch?v=cQ5XvudEdjE "PID Control")


##### Effects of each parameter:

Proportional - If its is too high the oscillations will keep on increasing and the system will not converge. If the parameter is too small, it will take a lot of time to get to the desired set point.

Integral - This parameter sets how much we take into consideration the cumulative error over time. This is usually used when we see that there is a systematic bias wherein we are not converging to our desired set point. So this parameter is used to drive the system towards that set point. 

Derivative - This parameter considers the rate of change in the error. If the error is rapidly approaching zero, this parameter will attempt to slow things down to avoid overshooting. 

In future, we can also use parameter optimization algorithms such as Twiddle to select optimal parameters.


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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 


