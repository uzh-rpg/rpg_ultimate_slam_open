# Installation of Ultimate SLAM

This installation guide was tested with Ubuntu 16.04 and Ubuntu 18.04.

### Requirements

- CMake >= 3.0
- ROS (>= Kinetic) (see [installation guide](http://wiki.ros.org/ROS/Installation)).

For Ceres, you need to install the following packages:

    sudo apt install liblapack-dev libblas-dev

### Install

First, we need to create a [catkin](http://wiki.ros.org/catkin) workspace for UltimateSLAM and initialize it:

    mkdir -p ~/uslam_ws/src && cd ~/uslam_ws
    catkin init

Then, we configure our workspace to extend ROS base workspace, and to compile in release mode (with optimizations) by default.
Please replace `kinetic` with your version of ROS (e.g. `melodic`).

    catkin config --extend /opt/ros/kinetic --cmake-args -DCMAKE_BUILD_TYPE=Release

Clone the UltimateSLAM repository and run `vcstool` to automatically import the dependencies:

    cd src/
    git clone git@github.com:uzh-rpg/rpg_ultimate_slam_open.git
    vcs-import < rpg_ultimate_slam_open/dependencies.yaml

Finally, build UltimateSLAM:

    catkin build ze_vio_ceres

This will take some time (a couple of minutes at least). In case of errors, see the Troubleshooting section below.

If building succeeds, congrats! You have installed UltimateSLAM. Your next step will be to [run some examples](Run-Examples.md) to test your setup.