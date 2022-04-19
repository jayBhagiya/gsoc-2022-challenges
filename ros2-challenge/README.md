
# ROS2 Challenge 

> Assumming you have installed docker on your machine.

- To Build the docker images and run the challenge, go through the below commands. For building the docker images, it may take around 30 mins.
```sh
cd gsoc-2022-challenges/ros2-challenge/ros2-codes/docker/
./build
```
- To run the image.
```sh
xhost + local:docker
./run foxy-main
```
- If you see the terminal changed, and the `tilde sign` then you are in a docker container. Now run the below commands inside the docker container to build our colcon workspace.
```sh
cd colcon_ws
colcon build --symlink-install
```
- Here colcon_ws and shared-dir are mounted volumes, so whatever changes we made inside a container or outside the host pc it will be synced in both container and host pc.

- #### Part 1.A - ROS2 Basics - Simple Listener / talker

   - To run a simple listener/talker node.
   ```sh
   # start new tmux session (inside docker container)
   tmux new -s ros2-basics
   
   # then press ctrl+b followed by | or - to split the terminal Vertical or Horizontal respectively.
   
   # now in one terminal run the listener node
   ros2 run pkg_ros2_basics listener
   # in other terminal run the talker node
   ros2 run pkg_ros2_basics talker
   ```
   
   **Solution Video - [Link][ros2-smpl-tl]**
   [![ROS2 Challenge Part 1.A](https://i.ytimg.com/vi/84K8SfmNC1k/maxresdefault.jpg)](https://www.youtube.com/watch?v=84K8SfmNC1k) 

- #### Part 1.B - ROS2 Basics - Visualization of laser data

   - To see the visualization of laser data in rviz2
   ```sh
   # start new tmux session (inside docker container)
   tmux new -s ros2-tb3
   
   # then press ctrl+b followed by | or - to split the terminal Vertical or Horizontal respectively.
   
   # now in one terminal launch the gazebo world with turtlebot3 robot
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py 
   # in other terminal launch the rviz
   ros2 launch turtlebot3_bringup rviz2.launch.py 
   # in another terminal run the teleop_keyboard node to teleop the turtlebot3
   ros2 run turtlebot3_teleop teleop_keyboard 
   ```
   
   **Solution Video - [Link][ros2-smpl-tb3]**
   [![ROS2 Challenge Part 1.B](https://i.ytimg.com/vi/7TcmbVt_RUI/maxresdefault.jpg)](https://www.youtube.com/watch?v=7TcmbVt_RUI) 

- #### Part 2

   - To navigate turltbot3 using simple nav goal and through waypoints. Follow the commands to launch the world and rviz2.
   ```sh
   # start new tmux session (inside docker container)
   tmux new -s ros2-nav
   
   # then press ctrl+b followed by | or - to split the terminal Vertical or Horizontal respectively.
   
   # In one terminal
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   
   # In other terminal
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/shared-dir/map/tb3_world.yaml
   ```
   - After launching the gazebo and rviz, you need to give a rough estination of the robot using `2D pose estimation` , after that you can send a simple nav using `Navigation2 Goal`.
   
   **Solution Video Simple Navigation - [Link][ros2-nav2-smpl]** 
   [![ROS2 Challenge Part 2.a](https://i.ytimg.com/vi/VG1O10qpMLY/maxresdefault.jpg)](https://www.youtube.com/watch?v=VG1O10qpMLY) 

   - For navigation through waypoints click on the `Waypoint Mode` in the left-down corner and give multiple `Navigation2 Goal`. After giving the navigation goals click on the `Start Navigation` in the left-down corner.
   
   **Solution Video Waypoints Navigation - [Link][ros2-nav2-wypt]**
   [![ROS2 Challenge Part 2.b](https://i.ytimg.com/vi/gbKVyj3APxs/maxresdefault.jpg)](https://www.youtube.com/watch?v=gbKVyj3APxs) 

[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format it nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)

   [ros2-smpl-tl]: <https://www.youtube.com/watch?v=84K8SfmNC1k>
   [ros2-smpl-tb3]: <https://www.youtube.com/watch?v=7TcmbVt_RUI>
   [ros2-nav2-smpl]: <https://www.youtube.com/watch?v=VG1O10qpMLY>
   [ros2-nav2-wypt]: <https://www.youtube.com/watch?v=gbKVyj3APxs>
