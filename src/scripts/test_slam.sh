#!/bin/sh

DIR="$( cd "$(dirname $(realpath $0))/../.." >/dev/null 2>&1 && pwd )"

xterm -hold -e  "
cd $DIR;
source devel/setup.bash;
export TURTLEBOT_GAZEBO_WORLD_FILE='$DIR/src/worlds/myoffice_v3.world';
roslaunch turtlebot_gazebo turtlebot_world.launch" &

sleep 5

xterm -hold -e "
cd $DIR;
source devel/setup.bash;
roslaunch turtlebot_gazebo gmapping_demo.launch" &

sleep 5

xterm -hold -e "
cd $DIR;
source devel/setup.bash;
roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 5

xterm -hold -e "
cd $DIR;
source devel/setup.bash;
roslaunch turtlebot_teleop keyboard_teleop.launch" &
