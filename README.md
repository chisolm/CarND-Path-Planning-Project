# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

---

[//]: # (Image References)

[image1]: ./writeup_images/50milerun.png "Long run with no incidence"
[image2]: ./writeup_images/partial_track.png "Show multiple tracks"
[image3]: ./writeup_images/best_particle_weight.png "Best particle weight"
[image4]: ./writeup_images/avg_distance.png "Obs to Landmark associated distance"

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

### Attribution

Portions of the code are from my class room exercises from the udacity class on self driving cars, behavior planning section and path planning walk-through.
The infrastructure code in this project is supplied from project repository listed above.

# Rubric Points

## Code Compilation

The code compiles cleanly with the existing cmake structure.

## 4.32 miles driven with no incidents

The car will very easily drive 4.32 miles without incidents.  I have let  
it run for about an hour and achieved a 51.55 mile run.  It does still have problems that are likely related to the wrapping of the s values at a full
track length.  It appears to have trouble less than 1 out of 5-10 laps.

![alt text][image1]

## Speed limit

The cars drives a target speed of 49.5 miles per hour unless there is a car ahead of
it blocking forward progress.

My implementation of speed control is not in the behavior state machine.  It is
still implemented as a direct control(reduction) of speed on sensing an obstacle.

## Max acceleration and Jerk

Max acceleration and jerk are never exceeded.  This is controlled by use of the
remaining points from previous path x and splines to continue the trajectory 
generation to next waypoints in the current lane or a lane change.

## Car does not have collisions

My car does not collide with other vehicles.

## Lane maintenance and lane transition

My car maintains it's lane unless it is executing a lane change.  Lane change
manuevers are maintained as a decision by a state in the state machine that 
continues a lane change through the end of its trajectory.  

My lane change decisions are via a cost model to transition to the lowest
cost lane.  This additional state avoids the problem of very close costs
causing multiple lane change (re)-decisions during a lane change maneuver
which can cause the car to exceed the 3 second lane change requirement.

## Ability to change lanes

My car evaluates the costs of lane change maneuvers every cycle and will 
changes lanes as the cost dictates.

# Reflection

### Code files

This is a short summary of what is in each *.cpp* file.  Discussion of the model and features will continue below:

main.cpp:  This file is the main JSON interface loop with the simulator.  It
also still contains the speed control for my vehicle with the ref_vel variable
and updates to the ego information in the road object.

helper.cpp:  This file contains all of the utility functions that were in the 
main.cpp file in the project repository.

trajectory.cpp:  This contains the method definitions for the trajectory
objects. It contains 2 methods for trajectory generation.  One for the ego 
car and another much simpler method for trajectory generation of the 
other cars on the road.

behavior.cpp:  This contains the behavior state machine and the cost functions 
that are used to evaluate the choice of the next state.

road.cpp:  This implements a singleton object for the road that keeps the map 
informatino, the ego information for my car and sensor for all the vehicles 
reported by the sensor fusion information.  It also contains a print method
that is very useful for debug, displaying the road, ego information and all
known cars.

vehicle.cpp:  This implements a simple object to keep track of traffic vehicles
reported by sensor fusion.

### Behavior Model

My behavior model centers around a state machine similar to the lane change 
state machine in the lessons.  I only use 3 primary states, "KL", "LCL" and
"LCR".  I also implement a lane change maneuver "LCM" state that has 
constrained entry from the lane change states.

I generate a single trajectory for each state and evaluate the costs of 
each for the next state choice.

The lane change maneuver state("LCM") was my solution to the problem of not
consistently choosing a lane change when the costs for keep lane and lane
change are nearly equal.  In prior versions my model, when costs were nearly
equal, would switch back and forth between lane change and keep lane.  This  
would lead to violations of the 3 second lane change requirement.  This is of
course of side effect of:
- continuing to use the existing path points in the trajectory(delaying lane change) 
- lack of another method to retain intent between behavior runs 
- the scoring method which evaluates the lane on the d value and will produce the same nearly equal cost values until the centerline is crossed.

This LCM state does retain the intent of changing lanes by fixing the desired lane for a period of time and constraining the next states while in LCM.

### Behavior state costs

To choose states in the behavior state machine I evaluate the costs for the 
given trajectory for each state with the following cost functions:

```
struct cost_functions {
  string name;
  function<double(Road, Trajectory, map<int, Trajectory>)> cf;
  double weight;
};

//Add additional cost functions here.
vector<cost_functions> const ncf_list = {
  {{"lane free"}, lane_most_free_ahead_cost, LANE_FREE_COST},
  {{"lane avail"}, available_slot, LANE_AVAIL_COST},
  {{"lane change"}, lane_change_cost, LANE_CHANGE_COST}
};
```

The lane_most_free_ahead_cost() function simply assigns a cost based on the 
closest vehicle in front of us.  It ends up giving a low or 0 cost to the 
lane with the most room to make forward progress.

The available_slot() evaluates the gap to make a lane change with a larger  
gap getting a smaller cost.  This is the cost function with the most potential
for improvement.  It uses a very large gap currently, leading to difficulty  
getting around a slow vehicle in heavy traffic.  It could use a significantly
smaller gap if it evaluated the trajectory of the surrounding cars for a 
crossing of the s values at any point in time in their trajectories.  The  
information was available, but limited was my time to implement it.

The lane_change_cost() is a small cost (relatively) to reduce the indecision  
on lane changes.  It does not completely eliminate it since the ego's 
evaluation of its lane location does not change until it crosses the mid-point
in the dividers.

### Trajectory generation

I used the trajectory generation from the walk through with the spline library.

The trajectory for the ego car is generated by taking the unused previous trajectory from the simulator and adding additional points to it based on the
spline waypoints.  Control for this trajectory is primarily via the supplied
desired lane for the trajectory and with velocity supplied(ref_vel).  The  
speed changes could be significantly improved by adding a current velocity 
and a desired velocity variables as seperate entities and generating velocity 
changes for each of the new trajectory points.

### Road object

In my implementation, this is primarily a container object to track the current 
state of the vehicle and surrounding cars.

The ego car position information could be moved out of there from a software 
engineering perspective.

From a path planning perspective, there is little going on in this object.

### Simulator improvement

As an assignment, this project would help by a simple change to the simulator.  If the collision information or acceleration and jerk information could be
passed back as part of the telemetry it would aid the debug process hugely.  I
think currently have a bug in my code that causes problems every 5-10 laps.  I
don't have a very useful way to debug that problem except for sitting and
watching the simulation.  At 25-50 minutes per cycle, that is just not
practical.  If the simulator supplied that information, I could log it and 
analyze the problem.




