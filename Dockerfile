# Use ROS 2 Iron as base image
FROM ros:iron

# Install required packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-iron-moveit \
    ros-iron-joint-state-publisher \
    ros-iron-robot-state-publisher \
    ros-iron-rviz2 \
    ros-iron-tf2-ros \
    ros-iron-xacro \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws/src

# Copy our pick and place package
COPY . /ros2_ws/src/pick_and_place

# Build the workspace
RUN /bin/bash -c '. /opt/ros/iron/setup.bash && colcon build'

# Source the workspace in .bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Set the entrypoint
COPY ./docker-entrypoint.sh /
RUN chmod +x /docker-entrypoint.sh
ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]