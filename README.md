# RBE 3002 Project Repo
## Running this project
### Partial Launch
Launching the AStar service is currently done through using `roslaunch rbe_3002 minimal.launch`.

I would recommend leaving the `minimal.launch` file as the basic launch for our package and adding additional launch files to test its integration with other packages.

### Full Launch
In order to run the full lab run `roslaunch rbe_3002 full.launch` This will launch `rviz` as well as our example script. The example script is on a 15 second delay to allow `rviz` to load fully before messages are sent.

## Clone Location
This project should be cloned into `catkin_wc/src/` so the directory should look like `catkin_wc/src/rbe_3002` afterward.

## Directory Layout
Please follow the [style-guide](http://wiki.ros.org/PyStyleGuide) for ROS python projects. This will make it so we don't have weird problems with catkin.

## Build project
From the `catkin_wc` directory run `catkin_make`.

## Testing
The test on the A Star Service can be run using `rostest rbe_3002 test-a-star.launch`
