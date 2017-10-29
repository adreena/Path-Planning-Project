# Path-Planning-Project

In this project car is given a trajectory to drive around the track without colliding with other cars and staying within speed limit.

The goals of this project are the following:

 * safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit
 * use car's localization, sensor fusion data and map list of waypoints around the highway  
 * having speed as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. 
 * The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. 
 * The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. 
 * Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

## Description
### Available Data

* Highway map: Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.
* Main car's localization Data (No Noise) [x, y, s, d, yaw, speed]. The x & y is car'sposition in map coordinates, s & d are cars' position in frenet coordinates, yaw is car's angle in the map, and finally the car's speed is in MPH.
* Sensor_fusion provides 2d vector of cars and then that car's with [unique ID, x, y ,vx, vy, s,d]
* Previous Path: ["previous_path_x"] & ["previous_path_y"] are the list of x & y points previously given to the simulator


## Approach

* I processed 3 previous (x,y) points of my car to generate my current acceleration and starting point of the trajectory: s_dot, d_dot , s_dot_dot & d_dot_dot. (using ["previous_path_x"] & ["previous_path_y"])

* For having a good estimate of my car's surrounding, I generated a prediction list of each cars' frenet coordinate (s,d) for next 5 frames (frame rate is 0.02 sec.) assuming that they each have a constant velocity. (using sensor_fusion data)

* Next step is deciding which state of state machine 3 states of "KL" as Keep Lane, "LCL" as Lane Change Left and "LCR" as Lane Change Right has the best trajectory (i.e. no collision, no jerk and low cost)

### Behavioral Planner

* Based on ego vehicle's current state I evaluate available states considering their trajectory and cost.
* Each available state is assumed to be a target state with boundary trajectories of target[s,s_dot, s_dot_dot, d, d_dot, d_dot_dot] and the goal is generating a continous trajectory from current state to target state with minimized trajectories of s and d using quintic polynomial, jerk-minimizing (JMT) trajectory, to estimate of the final trajectory based on the target state:
   * s(t) = a0+ a1t + a2t^2 + a3t^3 + a4t^4 + a5t^5
   * d(t) = b0+ b1t + b2t^2 + b3t^3 + b4t^4 + b5t^5
* Each trajectory cost needs to be evaluated to find the best option to transit from current state to:

   * collision cost:
   * speed cost
   * lane change cost


behaviors by specifying only a few quantities. For example by specifying only a target lane, a target vehicle (to follow), a target speed, and a time to reach these targets, we can make suggestions as nuanced as "stay in your lane but get behind that vehicle in the right lane so that you can pass it when the gap gets big enough."



1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.


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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```




