# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
## Introduction
This project implemented a path planning algorithm for a car driving on highway. 
It controls the speed, acceleration and the jerk within the limit. It also detects other vehicles, 
and makes decision to accelerate, decelerate, and change lanes based on other vehicles's location and speed.
The car tries to go as close as possible to the 50 MPH speed limit. It can finish one loop with any incidence
within around 5 minutes 30 seconds depending on the traffic.

The source code can be downloaded from [here](https://github.com/fwa785/CarND-Path-Planning-Project).

## Model

### Detection of other vehicles
The sensor fusion data provides information about the other cars. The first step of the algorithm is to find the
cars ahead and behind the vehicle. The distance and the speed of the cars ahead and behind this vehicle on all lanes
are recorded. 

          double car_ahead_s_delta[NUM_LANES] = { 1000, 1000, 1000 };
          double car_behind_s_delta[NUM_LANES] = { 1000, 1000, 1000 };
          double car_ahead_v[NUM_LANES] = { MAX_V, MAX_V, MAX_V };
          double car_behind_v[NUM_LANES] = { MIN_V, MIN_V, MIN_V };

          for (int i = 0; i < sensor_fusion.size(); i++) {
            // get the car's lane
            double d = sensor_fusion[i][6];
            int    lane = d2lane(d);

            double vx = sensor_fusion[i][3]; // velocity on x
            double vy = sensor_fusion[i][4]; // velocity on y
            double v = sqrt(vx * vx + vy * vy); // total velocity
            double s = sensor_fusion[i][5]; // s in Frenet

            // predict where this car will be after prev_size steps
            s += v * DELTA_T * prev_size;

            // Detect the car right in front of this car
            if ((s > car_s) && ((s - car_s) < car_ahead_s_delta[lane])) {
              car_ahead_s_delta[lane] = s - car_s;
              car_ahead_v[lane] = v;
            }

            // Detect the car right after this car
            if ((s < car_s) && ((car_s - s) < car_behind_s_delta[lane])) {
              car_behind_s_delta[lane] = car_s - s;
              car_behind_v[lane] = v;
            }
          }

### Select the target lane
In addition, the car with the highest velocity among all the lanes is detected, and the lane is set as
the target lane for this vehicle to change to. However, if the speed of the car ahead on another lane is 
slower than the car ahead on the current lane, but the car on the other lane is far more ahead than this car
on the current lane, we also choose the other lane as the target lane to change. The reason is the car on the
other lane is far, so we can change to the other lane temporarily and then have enough space to switch back to 
the current lane to bypass the slow car in front of us. Sometimes the target lane is 2 lanes away, then the target lane is 
adjusted to be the next lane can help to get to the target.

          /* Find out the target lane to change, target lane maybe two lanes away */
          int target_lane = cur_lane;
          if (car_ahead_s_delta[cur_lane] < (TRAILING_DIST * 3)) {
            int max_v = car_ahead_v[cur_lane];
            //int start_lane = rand() % NUM_LANES;
            int start_lane = 0;
            for (int i = 0; i < NUM_LANES; i++) {
              int check_lane = (start_lane + i) % NUM_LANES;
              if ( (car_ahead_s_delta[check_lane] > (car_ahead_s_delta[cur_lane] + TRAILING_DIST * 2)) ||
                   (car_ahead_v[check_lane] > max_v) )
              {
                max_v = car_ahead_v[check_lane];
                target_lane = check_lane;
              }
            }
          }
  
          if (target_lane > cur_lane) {
            target_lane = cur_lane + 1;
          }
          else if (target_lane < cur_lane) {
            target_lane = cur_lane - 1;
          }


### Plan the path

First the algorithm takes the last two points from the previous planned path. This way, we can make sure the path of
the car is continous from the previous planned path. The heading angle of the car is also calculated from the previous
path.

          if (prev_size < 2)
          {
            px = car_x;
            py = car_y;
            psi = deg2rad(car_yaw);

            px_prev = px - cos(psi);
            py_prev = py - sin(psi);
          }
          else {
            px = previous_path_x[prev_size - 1];
            py = previous_path_y[prev_size - 1];

            px_prev = previous_path_x[prev_size - 2];
            py_prev = previous_path_y[prev_size - 2];
            psi = atan2(py - py_prev, px - px_prev);
          }

          ptsx.push_back(px_prev);
          ptsy.push_back(py_prev);

          ptsx.push_back(px);
          ptsy.push_back(py);


Then 3 path positions are planed on the Frenet coordinate with s of the points on the coordinate aparted by 30 meters.
While generating the 3 path positions, we also decide whether we change lanes starting from the second position. If the
target lane chosen is different than the current lane, and there are enough space before and after my car, we can change
the lane, and plan the second and third positions on the new lane. In order to prevent too frequent lane changes, I intrdouced
a variable stay_on_lane to count the number of cycles the vehicle stay on the current lane. It can only change to a different 
lane after staying on this lane for certain extended period.

          // generate 3 positions on Frenet coordinate for smooth 
          // fitting of the planned path
          for (int i = 1; i <= 3; i++) {

            double next_s = car_s + 30 * i;
            double next_d = lane2d(cur_lane);

            vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(xy[0]);
            ptsy.push_back(xy[1]);

            /* change lane */
            if (i == 1) {
              stay_on_lane++;

              if ( (stay_on_lane > 100) &&
                   (target_lane != cur_lane) )
              {
                if ((car_ahead_s_delta[target_lane] >= TRAILING_DIST) &&
                    (car_behind_s_delta[target_lane] >= BYPASS_DIST))
                {
                  cur_lane = target_lane;
                  stay_on_lane = 0;
                }
              }
            }
          }

### Generate smooth path
With the 5 points we generated, we can use the spline library to help us generate smooth and continues path along these
5 points.

To do that, we first need to convert the coordinate from the map coordinate to the car's coordinate. The reason is, on the
car's coordinate, the car is aheading to the X axis, and x always increases. The spline library can only fit a curve with
incremental x. Another reason to choose car's coordiante is later on, when we generate more path points from the spline function,
we can have a better control of x variable. Since x axis is the direction the car's driving, we can generate x based on the car's
velocity and the time interval(0.02s), and then use the x value to generate the y value based on the spline fitting.

          // Now convert the points from map coordinate to the car's coordinate
          // because it's easier to generate middle points on the car's coordinate
          // with controlled speed
          for (int i = 0; i < ptsx.size(); i++) {
            double x_diff = ptsx[i] - px;
            double y_diff = ptsy[i] - py;

            ptsx[i] = x_diff * cos(-psi) - y_diff * sin(-psi);
            ptsy[i] = y_diff * cos(-psi) + x_diff * sin(-psi);
          }

          tk::spline s;
          s.set_points(ptsx, ptsy);

### Generate the points on the planned path
Finally we can generate the points on the planned path. 

First, we add the points not used by the previous path to this path to guarantee a continous path from previous path.

          // use the left over points in the previous path 
          for (int i = 0; i < prev_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

Second, we generate the points from the spline fitting. We will generate the points on each segments of 30 meters apart on
car's coordiante's x axis. With the target x and y value, we can caculate the target distance for each segment.

          double target_x = 30;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);

The target distance can be useful to caculate the x axis difference on each 0.02 second interval based on the current velocit 
of the vehicle. With that, the difference of y can be caculated, and the point is converted back to the map coordiante for
the simulator to use as the points on the next path.

            double N = target_dist / (mph2mps(ref_v) * DELTA_T);            
            diff_x += target_x / N;
            double diff_y = s(diff_x);
            // convert the x, y from car's coordinate back to the map's cooridnate
            double next_x = px + diff_x * cos(psi) - diff_y * sin(psi);
            double next_y = py + diff_x * sin(psi) + diff_y * cos(psi);

            next_x_vals.push_back(next_x);
            next_y_vals.push_back(next_y);
			

After each point is generated, we need to check how to accelerate or decelerate the vehicle. 

We first update the distance of the cars ahead of us on the target lane the current lane, based on their velocity and
our velocity. If both the cars on the current lane and the target lane has enough has enough distance ahead of us, we
can accelerate to catch up. However, if the car on the target lane doesn't have enough distance ahead of us, and it's
even smaller than the distance we can bypass it, then we also accelerate to try to run ahead of it to bypass it.

            /* match the speed to the target lane */
            if ( ((car_ahead_s_delta[target_lane] > TRAILING_DIST) ||
                   (car_ahead_s_delta[target_lane] < BYPASS_DIST)) &&
                 (car_ahead_s_delta[cur_lane] > TRAILING_DIST) )
            {
              ref_v += acc;
            }

If the car can't accelerate, check whether it needs to decelerate to match the speed on the target lane in order to change
lane. But it can't decelerate to be even slower than the car right behind us. 

            else {
              if (ref_v - dec > car_behind_v[target_lane]) {
                ref_v -= dec;
              }
              else {
                ref_v = car_behind_v[target_lane];
              }

			  /* Has to at least faster than the car behind use */
              if (car_behind_s_delta[cur_lane] < BYPASS_DIST) {
                if (ref_v < car_behind_v[cur_lane]) {
                  ref_v = car_behind_v[cur_lane];
                }
              }
            }

## Result
With the path planning algorithm described above, the vehicle can at least drive along the highway with one loop at the speed as close to
50MPH as possible without any incidence.
			
[![IMAGE Final Result](https://img.youtube.com/vi/XJq7NK-AkQo/0.jpg)](https://www.youtube.com/watch?v=XJq7NK-AkQo)
			
## Build Instructions

1. Clone the repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

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


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
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

