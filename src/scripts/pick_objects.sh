#!/bin/sh

DIR="$( cd "$(dirname $(realpath $0))/../.." >/dev/null 2>&1 && pwd )"

xterm -e  "
cd $DIR;
source devel/setup.bash;
export TURTLEBOT_GAZEBO_WORLD_FILE='$DIR/src/my_robot/worlds/myoffice_new.world';
roslaunch turtlebot_gazebo turtlebot_world.launch" &

sleep 5

xterm -e "
cd $DIR;
source devel/setup.bash;
export TURTLEBOT_GAZEBO_MAP_FILE='$DIR/src/my_robot/maps/myoffice.yaml';
roslaunch turtlebot_gazebo amcl_demo.launch" &

sleep 5

xterm  -e "
cd $DIR;
source devel/setup.bash;
roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 5

xterm  -e "
cd $DIR;
source devel/setup.bash;
rosrun pick_objects pick_objects" &

