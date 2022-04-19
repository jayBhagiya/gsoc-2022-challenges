# GSoC'22 Jderobot Challenges

All four challenges are solved and codes and problem statements(in pdf) can be found in their respective folders. Below are the four challenges.
- [CPP Challenge][cpp-cln]
- [Python Challenge][py-cln]
- [ROS2 Challenge][ros2-cln]
- [Robotics Academy Challenge][ra-cln]

## Installation

For running ROS2 and Robotics Academy Challenge, You need to install docker on your pc. Check here for docker [installation][docker-install]

- Clone the repository.
```sh
git clone https://github.com/jayBhagiya/gsoc-2022-challenges.git
```

## 1. CPP Challenge - Labyrinth Solver

- Input maze for the Labyrinth is given in the input.txt file and the solution for Labyrinth will be stored in the output.txt file. Run the below commands to build and run code. 
```sh
cd gsoc-2022-challenges/cpp-challenge/labyrinth-solution-2022/
./build.sh
```

## 2. Python Challenge - Brownian Motion

- To run the simulation, see the below commands. To exit, the simulation simply hit the space button.
```sh
cd gsoc-2022-challenges/python-challenge/brownian-robot-solution/
python3 main.py
```

**Solution Video - [https://www.youtube.com/watch?v=lA0oALM7dOM][py-cln-vd]**

## 3. ROS2 Challenge 

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
- Now colcon_ws and shared-dir are mounted volumes, so whatever changes we made inside a container or outside the host pc it will be synced in both container and host pc.

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
   
   **Solution Video - [https://www.youtube.com/watch?v=84K8SfmNC1k][ros2-smpl-tl]**

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
   
   **Solution Video - [https://www.youtube.com/watch?v=7TcmbVt_RUI][ros2-smpl-tb3]**

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
   
   **Solution Video Simple Navigation - [https://www.youtube.com/watch?v=VG1O10qpMLY][ros2-nav2-smpl]** 
   
   - For navigation through waypoints click on the `Waypoint Mode` in the left-down corner and give multiple `Navigation2 Goal`. After giving the navigation goals click on the `Start Navigation` in the left-down corner.
   
   **Solution Video Waypoints Navigation - [https://www.youtube.com/watch?v=gbKVyj3APxs][ros2-nav2-wypt]**


## 4. Robotics Academy Challenge - Vacuum Cleaner

- To get detailed Information about vacuum cleaner exercise, follow the [link][vc-link]. Exercise is solved using the `Random Coverage Algorithm`. Below are the commands to run the exercise.
```sh
cd gsoc-2022-challenges/robotics-academy-challenge/docker/
./run jderobot/robotics-academy:3.1.6
```
- Now go to the web browser and paste the URL, that you will got on terminal output. After that go to the vacuum cleaner exercise. Click on the connect and launch button. Once the `Connection Established` window pops up, you can start solving the exercise.
- The solution Code is given at `gsoc-2022-challenges/robotics-academy-challenge/Solution/main.py`. Just copy-paste code from main.py to the web template inside the web browser. At the end click on the start button to start the exercise.

**Solution Video - [https://www.youtube.com/watch?v=x1Pm9YwRxw0][ra-vcl]**

[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format it nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)

   [cpp-cln]: <https://drive.google.com/file/d/1GO0GJIi7rNqZXhPEaV8Qf0Ds4qFHczJ2/view?usp=sharing>
   [py-cln]: <https://drive.google.com/file/d/1Mzr-jGvwCpuZoFKmvjXzJxTfjbf2K16w/view?usp=sharing>
   [ros2-cln]: <https://drive.google.com/file/d/1q3-43jxPPqQRjRB_tX7Y_N07YKOfToC3/view?usp=sharing>
   [ra-cln]: <https://drive.google.com/file/d/1UaZt43_Sl-vI-mD_D6bRNWXLQpywdrIF/view?usp=sharing>
   [docker-install]: <https://docs.docker.com/engine/install/>
   [vc-link]: <http://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/vacuum_cleaner>

   [py-cln-vd]: <https://www.youtube.com/watch?v=lA0oALM7dOM>
   [ros2-smpl-tl]: <https://www.youtube.com/watch?v=84K8SfmNC1k>
   [ros2-smpl-tb3]: <https://www.youtube.com/watch?v=7TcmbVt_RUI>
   [ros2-nav2-smpl]: <https://www.youtube.com/watch?v=VG1O10qpMLY>
   [ros2-nav2-wypt]: <https://www.youtube.com/watch?v=gbKVyj3APxs>
   [ra-vcl]: <https://www.youtube.com/watch?v=x1Pm9YwRxw0>
