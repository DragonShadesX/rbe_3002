# RBE 3002 Project Repo
## Clone Location
This project should be cloned into `catkin_wc/src/` so the directory should look like `catkin_wc/src/rbe_3002` afterward.

## Directory Layout
Please follow the [style-guide](http://wiki.ros.org/PyStyleGuide) for ROS python projects. This will make it so we don't have weird problems with catkin.

## Build project
From the `catkin_wc` directory run `catkin_make`.

## Running this project
Launching the AStar service is currently done through using `roslaunch rbe3002 minimal.launch`.

I would recommend leaving the `minimal.launch` file as the basic launch for our package and adding additional launch files to test its integration with other packages.