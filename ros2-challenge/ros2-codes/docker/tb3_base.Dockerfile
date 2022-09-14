# foxy-wrapper image with ros2/foxy and tools installed
FROM ros2/foxy:1.0

# Create a colcon workspace and clone TurtleBot3 repos
RUN echo "ubuntu\n" | sudo -S apt-get update -y \
  && echo "ubuntu\n" | sudo -S apt-get install -y \
    ros-foxy-turtlebot3 \
    ros-foxy-turtlebot3-msgs

# creating colcon_ws
RUN mkdir -p ~/colcon_ws/src \
  && cd ~/colcon_ws/src 

# mount the local workspace to container
COPY ./colcon_ws/src/ ~/colcon_ws/src/

# Build the Catkin workspace and ensure it's sourced
RUN source /opt/ros/foxy/setup.zsh \
  && cd ~/colcon_ws \
  && colcon build --symlink-install
