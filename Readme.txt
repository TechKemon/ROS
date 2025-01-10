# to build docker image,
docker build -t ros2_pick_and_place .
docker run -it --rm ros2_pick_and_place

# Inside the new container:
cd /ros2_ws
colcon build
source /opt/ros/iron/setup.bash
source install/setup.bashcd /ros2_ws
colcon build
source /opt/ros/iron/setup.bash
source install/setup.bash

# the robot is up & running now
