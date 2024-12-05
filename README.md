# Lab 6: SLAM and Pure Pursuit

### Launch Command
```bash
ros2 launch pure_pursuit lab5_launch.py mode:=v logging:=false file:=/sim_ws/src/pure_pursuit/src/AEBFlexwaypoints3.csv mxangle:=35.0 speed:=1.0


ros2 launch pure_pursuit lab5_launch.py mode:=sim logging:=false file:=/sim_ws/src/pure_pursuit/src/AEBwaypoints.csv mxangle:=35.0 speed:=2.0

ros2 launch pure_pursuit lab5_launch.py mode:=v logging:=false file:=/home/f1/Joseph_et_al/src/CS489Lab5-main/pure_pursuit/src/AEBwaypoints2.csv mxangle:=20.0 speed:=4.0 l:=1.0
```
#### Parameters:
`file` = filepath to .csv file with waypoint information (default is `/sim_ws/src/pure_pursuit/src/waypoints.csv`)

`mode` = set to either `sim` for simulator or `v` for physical vehicle

`logging` = set to `true` if logging waypoint information

`l` = maximum lookahead of waypoints for the vehicle

`speed` = maximum speed of vehicle

`mxangle` = maximum steering angle of vehicle

## I. Learning Goals

- SLAM
- Localization with Particle Filter
- Pure Pursuit Algorithm

## II. Running slam_toolbox in Simulator

Follow the instructions provided in the *sim_instruction* pdf to run `slam_toolbox` to make a map of Gazebo's waffle map. Save the map as `waffle.pgm` and `waffle.yaml`.

## III. Localization with Particle Filter in Simulator

Follow the instructions given in the pdf to run `particle_filter` in simulator using the Levine map.

## IV. Pure Pursuit Implementation

We have provided a skeleton for the pure pursuit node. As per usual, test your algorithm first in the simulator before you test it on the car. When you're testing in the simulator, use the ground truth pose provided by the sim as the localization. When you move to the car, use particle filter to provide localization.

As shown in the lecture, the curvature of the arc to track
can be calculated as:

<!-- ![](https://latex.codecogs.com/svg.latex?\gamma=\frac{2|y|}{L^2}) -->
$$\gamma=\frac{2|y|}{L^2}$$

## V. Logging Waypoints

There are several methods you can use to create waypoints for a specific map.

1. Recording a trajectory of joystick driven path. You can write a node that subscribe to the pose provided by the particle filter localization, and save the waypoints to a csv file. A similar script is provided [here](https://github.com/f1tenth/f1tenth_labs/blob/main/waypoint_logger/scripts/waypoint_logger.py). Note that this script is in ROS 1 and you'll have to write a ROS 2 node.

2. Find key points in the map (e.g. in the Levine loop, the four corner centers of the loop) and create a interpolated spline that goes through all four corners. You can use functions such as `scipy.interpolate.splprep` and `scipy.interpolate.splev`. You can find more documentaion on these [here](https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.splprep.html) and [here](https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.splev.html#scipy.interpolate.splev).

Usually, you'll just save the waypoints as `.csv` files with columns such as `[x, y, theta, velocity, arc_length, curvature]`. With pure pursuit, the bare minimum is `[x, y]` positions of the waypoints. Another trick is that you can also smooth the waypoints if you decided to record it with the car. You can subsample the points you gathered and re-interpolate them with the `scipy` functions mentioned above to find better waypoints.

## VI. Visualizing Waypoints

To visualize the list of waypoints you have, and to visualize the current waypoint you're picking, you'll need to use the `visualization_msgs` messages and RViz. You can find some information [here](http://wiki.ros.org/rviz/DisplayTypes/Marker).

There also exists interactive markers that allows students to manually edit waypoints in simulation. Consider looking into the documentation. 

## VII. Deliverables

- **Deliverable 1**: Submit the map files (waffle.pgm and waffle.yaml) that you've made using `slam_toolbox`.
- **Deliverable 2**: Commit your pure pursuit package to GitHub. Your commited code should run smoothly in simulation. Students will be required to utilize a launch file to launch their 
nodes for mapping and waypoint generation. 

Share your git repository with your TA set as a collaborator. The TA will pull your repository to the car. Maps will be provided for physical tracks 
through canvas. Students will recieve *'.pgm'* and *'.yaml'* files to add waypoints to.


## VIII: Grading Rubric
- Compilation: **10** Points
- Running slam_toolbox and producing a map: **30** Points
- Running particle_filter: **20** Points
- Implementing pure pursuit: **40** Points

