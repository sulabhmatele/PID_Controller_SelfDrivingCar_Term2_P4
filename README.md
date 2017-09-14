# PID_Controller_SelfDrivingCar_Term2_P4
This repo contains the submissions and related material for Udacity "Self Driving Car" Nano degree program's Term 2 - Project 4, "PID_Controller"

###Introduction

This project implements the basic PID controller, which is used to control the steering wheel angle and also the speed of the car, based on the cte (cross talk error) provided by the simulation.
The implementation logic is concentrated on tuning the 3 parameters :

* Kp - Proportional parameter
* Kd - Differential parameter
* Ki - Integral parameter

### Relation of each param
All 3 parameters has the direct effect on the steering wheel angle, where:

* Kp - Has the most impact on change, so if the value of Kp is increased, it allows the change on steering wheel angle very rapidly.
* Kd - It's is parameter which allows the car to get in control and keeps car on track and resists the sudden change. This value is generally larger than Kp.
* Ki - This parameter negates to any error over the period of time, which was not allowing car to reach on desired value on time. We need to be very careful and keep the value very low.   

The main relation between parameters, which helped me was: 

Kp = 0.1 * Kd

Ki = 0.1 * Kp

* Thanks to this post for helpful information on this topic 

    [StackExchangeDiscussionPID](https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops) 

### Parameter tuning
For parameter tuning I have used 2 methods, by manually tuning the parameters and seeing the output on simulator, which allows me to start with a stable set of values.

Next part was to make car adjusting these parameters based on requirements on track. For allowing car to make change in the hyper parameters, I have used the Twiddle algorithm.

Twiddle algorithm, allows the parameters to change when the error increases by some amount.
I found the following values working good for me as result of manual tuning and used as good start parameters.

```
Kp = 0.4;
Ki = 0.00001;
Kd = 4;
```

Twiddle Algorithm - (As described in Udacity lesson)

```
def twiddle(tol=0.2): 
    p = [0, 0, 0]
    dp = [1, 1, 1]
    robot = make_robot()
    x_trajectory, y_trajectory, best_err = run(robot, p)

    it = 0
    while sum(dp) > tol:
        print("Iteration {}, best error = {}".format(it, best_err))
        for i in range(len(p)):
            p[i] += dp[i]
            robot = make_robot()
            x_trajectory, y_trajectory, err = run(robot, p)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                robot = make_robot()
                x_trajectory, y_trajectory, err = run(robot, p)

                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
        it += 1
    return p
```


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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