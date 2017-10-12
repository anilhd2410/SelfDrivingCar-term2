# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
## Implementation details:

PID simultor gives us the state of the car with respect to the track can be seen in the simulator which
is called as CTE(cross track error), CTE means how far is the car from the track center.
In Additonal it also gives us the heading error(angle) and the speed

Goal of the implementation is to calculate the correct steering value.

To do this I implemented 3 functions in PID class:

* Init
Set Initial PID scalars / weights

* UpdateError 
Update proportional, differential and integral errors

  * The Proportional error is the new CTE
  * The integral errors is the sum of all the CTE 
  * The differential error is the difference between current and previous CTE


* Total error

Total error is calculated using below formula which multiplying PID scalars /weights with corresponding component/term
    ```
    Total error = Kp * p_error + Ki * i_error + Kd * d_error
    ```

### Components or terms

  * P term (proportional) :
  The proportional term drives the error to zero, however, these results in error oscillating about the set point.

  * I term (Integral) :The integral term, pushes the control in the opposite direction of the accumulated error, which can pull right or left.

  * D term (derivative) :The derivative term, the differential from previous to current error, to handle overshoot around the centerline


### PID Tuning:
Parmeters are tuned manually or Visually and kept constant throttel

I fixed throttle at 0.3

And final scale values are:

***Pid_VarKp = -0.09, 	
Pid_VarKi = -0.0009, 
Pid_VarKd = -4.0***

* First step: 	
```
K_P = -1.0, since the steering is between -1 to 1 
K_I = 0
K_D = -1, negative value to minimize the amount of change 
```

Observed too much oscillation and car travels to out of track to the right immediately,
So I reduces K_P and K_D vales 

* Second step and later:	
```
K_P = -0.5, 
K_I = 0
K_D = -2, 
```

Observed improvement and car passed successfully first curve and goes out of the track, and then I continued to decreasing K_P to reduce the oscillation and K_D helped to pullback the car on to the center
In addition, I observed car drifting to the right so added small value to K_I, which helped center the car.

## Improvements required:
*	Since K_D was set too high, in curves can observe quick and extreme steering changes
*	Car drives close to the edge near last right turn of the lap 
*	Managing throttle and speed
*	Using Twiddle algorithm


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

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

