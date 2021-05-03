# ROS-COMPSYS726-Turtlebot2
## This project will move the turtlebot2 robot autonomously from one side of a random room to another. It will avoid any obstacles whilst taking photos of obstacles it encounters.

### Assumptions for room:
- Possible to reach other side of room
- Single door in room
- Walls are continous, no other gap than door
- Maximum of 10 static obstacles
- Robot will start facing wall with the door

To build project `run catkin_make`
Then `cd` into A2 folder and run `source/devel/setup.bash` 
To run project run `roslaunch python_nodes python.launch`
