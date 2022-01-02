# Home Service Robot
***
This is the last project of the Udacity Nanodegree Program "Robotics Software Engineer". Throughout this project a map shall be created through SLAM. The map shall be used as for navigation of the Turtlebot. In the end, the robot shall be able to navigate to a pickup-zone to pick up a virtual object (marker) and drop it off in a drop-off-zone.
***
## First steps
Step 1: Update the system
`sudo apt-get update`
Step 2: Install the ROS navigation stack
`sudo apt-get install ros-kinetic-navigation`
Step 3: Copy the folder catkin_ws into your workspace
Step 4: Change directory to catkin_ws
```
$ cd (yourPath)/catkin_ws
$ catkin_make
```
(Optional step: Install rospkg)
```
$ pip install rospkg
$ catkin_make
```
Step 5: Source the project
`source devel/setup.bash`
***
## Included Packages
- gmapping
- turtlebot_teleop
- turtlebot_rviz_launchers
- turtlebot_gazebo
- my_robot
- add_markers
- pick_objects

### GitHub directories to be cloned
The gmapping package is part of https://github.com/ros-perception/slam_gmapping.git
The turtlebot_teleop package is part of https://github.com/turtlebot/turtlebot.git
The turtlebot_rviz_launchers is part of https://github.com/turtlebot/turtlebot_interactions.git
The turtlebot_gazebo is part of https://github.com/turtlebot/turtlebot_simulator.git
The four were cloned into the workspace.
***
### gmapping
The gmapping package includes a particle filter for grid maps created with laser range data. It therefore enables the robot to perform SLAM. We will use the gmapping_demo.launch file to perform SLAM and build a map of the environment either with laser range finder sensors or RGB-D cameras.
### turtlebot_teleop
The turtlebot_teleop package enables us to manually control the robot with keyboard commands. We will be using the keyboard_teleop.launch file.
### turtlebot_rviz_launchers
The turtlebot_rviz_launchers package allows us to load a rviz workspace. We will be using the view_navigation.launch which loads a preconfigured rviz workspace with our robot model, trajectories and the map.
### turtlebot_gazebo
The turtlebot_gazebo package enables us to load the tutlebot into our gazebo environment. We will be using the turtlebot_world.launch and link our world file to it.
### my_robot
The my_robot package is a package imported from the previous project. We will be using the world file and save our map in that package.
### pick_objects
This package is self-created and enables the robot to move to the pickup location by sending goals to the navigation stack. The navigation stack then creates a path to the goal. When the robot arrives at the pickup-zone, the robot publishes that it reached the zone. It will wait for 5 seconds to "pick up" the object. It will then move to the drop-off-location and then publishes that it arrived at this zone. It will then drop off the object.
### add_markers
This package is self-created and creates a visual marker (green square) in the rviz workspace. The marker represents the object to pick up / drop off. The executable gets the information about the arrival of the robot at the pickup/drop-off zone by subscribing to the topics published by pick_objects.
***
## Scripts
### test_slam.sh
The script enables the user to move in the environment and creating a map by using SLAM. This script was used to create the map for the following scripts. The gmapping_demo.launch (gmapping) is used for SLAM and the turtlebot_teleop package for manually controlling the robot.
### test_navigation.sh
A 2D Nav goal (in rviz workspace) can be communicated to the robot so that it moves to the marked location. The localization is enabled by the amcl_demo.launch from the turtlebot_gazebo package.
### pick_objects.sh
The script executes the pick_objects node and extends the functionalities of test_navigation.sh. Thus, the robot is given two goals (pickup and dropoff location) which it is moving to with a 5 seconds pause after reaching the first goal.
### add_marker.sh
A marker at the pickup zone is published. After 5 seconds the marker is hidden (deleted). After another 5 seconds, the marker is published at the dropoff zone.
### home_service.sh
The script extends the pick_objects shell script. Additionally to the functionalities of the pick_objects.sh, the picked up / dropped off object is embodied by a marker (green square).
