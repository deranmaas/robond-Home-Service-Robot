# Home Service Robot

![navigation](./images/navigation.png)

## A brief description of the used packages

### turtlebot_gazebo

The [turtlebot_gazebo](http://wiki.ros.org/turtlebot_gazebo) package contains launchers to deploy a turtlebot in a
gazebo environment by linking the world file to it.

### turtlebot_rviz_launchers

The [turtlebot_rviz_launchers](http://wiki.ros.org/turtlebot_rviz_launchers) package contains the view_navigation.launch
file to load a preconfigured rviz workspace. It will automatically load the robot model, trajectories, and a map.

### turtlebot_teleop

The [turtlebot_teleop](http://wiki.ros.org/turtlebot_teleop) package provides teleoperation using joysticks or keyboard.

### gmapping (SLAM)

The [gmapping](http://wiki.ros.org/gmapping) package contains a ROS wrapper for OpenSlam's Gmapping. It provides laser-based SLAM (Simultaneous Localization and Mapping), which will be used to create a 2-D occupancy
grid map from laser and pose data collected by the mobile robot.

### amcl (localization)

The [amcl](http://wiki.ros.org/amcl) package amcl is a probabilistic localization system for a robot moving in 2D. It
implements the adaptive (or KLD-sampling) Monte Carlo localization approach, which uses a particle filter to track the
pose of a robot against a known map.

### move_base (navigation)

The [move_base](http://wiki.ros.org/move_base) package creates a path for the robot while avoiding obstacles on its
path.

### pick_objects (own development)

Package that will send multiple goals for the robot to reach. The robot travels to the desired pickup zone, displays a
message that it reached its destination, waits 5 seconds, travels to the desired drop off zone, and displays a message
that it reached the drop off zone."

### add_markers (own development)

This package will model a virtual object with markers in rviz. Initially it will show the marker at the pickup zone and
hide the marker once the robot reaches the pickup zone. Finally, it will show the marker at the drop off zone once the
robot reaches it.
