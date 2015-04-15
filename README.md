# RBE 3002 Project Repo
## Running this project
## Automatic Code Launching (Local Map)
This will launch everything so that you can test that the whole system works: `roslaunch rbe_3002 local_map_test.launch`

## Launching Nav on the Robot
On Robot: `roslaunch turtlebot_bringup minimal.launch` and `roslaunch turtlebot_navigation gmapping_demo.launch`. On your local computer: `roslaunch rbe_3002 robot_map_test.launch` Then run `rosrun rbe_3002 lab4.py` to path plan using the map the robot has generated.

## Manual Launch Configuration (Local Map)
Launch list (each in different terminal) `roscore`, `rosrun rviz rviz -d {config file}`, `rosrun rbe_3002 map_scaler.py`, `rosrun rbe_3002 map_obsticle_expander.py`, `rosrun rbe_3002 a_star_server.py`, `rosrun map_server map_server {map file}`, `rosrun rbe_3002 lab4.py`
Every time you want to change a_star_server code you only need to re-run the a_star server and the map_server (in that order). The map transformer nodes will only run if the map_server rebroadcasts a message.

### Partial Launch (Not tested on current version)
Launching the AStar service is currently done through using `roslaunch rbe_3002 minimal.launch`.

I would recommend leaving the `minimal.launch` file as the basic launch for our package and adding additional launch files to test its integration with other packages.

### Full Launch (Not tested on current version)
In order to run the full lab run `roslaunch rbe_3002 full.launch` This will launch `rviz` as well as our example script. The example script is on a 15 second delay to allow `rviz` to load fully before messages are sent.

## Clone Location
This project should be cloned into `catkin_wc/src/` so the directory should look like `catkin_wc/src/rbe_3002` afterward.

## Directory Layout
Please follow the [style-guide](http://wiki.ros.org/PyStyleGuide) for ROS python projects. This will make it so we don't have weird problems with catkin.

## Build project
From the `catkin_wc` directory run `catkin_make`.

## Testing
The test on the A Star Service can be run using `rostest rbe_3002 test-a-star.launch`
