# ucsd_robocar_hub2

<img src="ucsd_ros_logo.png">

This is the UCSD robocar hub metapackage. It contains packages for our sensors, actuators, independent path planning algorithms and the navigation package that functions as the "brain" by being able to call all of the other packages and do some interesting things by combining multiple types of path planning and the sensors used to accomplish this. All packages are written in **ROS2** and have been tested on **Foxy**.

# Getting Started 
1. Its suggested to just pull the <a href="https://hub.docker.com/repository/docker/djnighti/ucsd_robocar" >docker image</a> for this meta package for faster development instead of having to install all the (many) dependencies required to get this metapackage working.
1. Next, go to the <a href="https://gitlab.com/ucsd_robocar/ucsd_robocar_nav2_pkg" >ucsd_robocar_nav2_pkg</a> and start with the README.md as it explains in detail how it can communicate to all of the other packages, has step-by-step instructions for getting the camera navigation working and plenty of other details.
1. Each package has its own dedicated README.md that explains in more detail how it works and what launch/nodes/topics it has to offer. To get to each of the packages, just click on any of the submodules above. 
