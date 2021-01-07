# Run Examples

In this page, we will show how to run UltimateSLAM offline on a few datasets from the [Event Camera Dataset](http://rpg.ifi.uzh.ch/davis_data.html).

### Download datasets

First, make sure UltimateSLAM is sourced in your current terminal:

```
source ~/uslam_ws/devel/setup.bash
```

Now, navigate to the `ultimate_slam` folder and create a `data` folder:

```
roscd ze_vio_ceres/../../
mkdir data
cd data/
```

Download some example datasets:

```
wget http://rpg.ifi.uzh.ch/datasets/davis/boxes_6dof.bag
wget http://rpg.ifi.uzh.ch/datasets/davis/dynamic_6dof.bag
wget http://rpg.ifi.uzh.ch/datasets/davis/shapes_6dof.bag
```

### Run UltimateSLAM

UltimateSLAM can run in two different modes: either using only events and IMU, or using events, frames and IMU.

#### Using events + IMU

```
roslaunch ze_vio_ceres ijrr17_events_only.launch bag_filename:=dynamic_6dof.bag
```

#### Using events + frames + IMU

```
roslaunch ze_vio_ceres ijrr17.launch bag_filename:=dynamic_6dof.bag
```

When launching, an RVIZ window should pop up, showing the current estimated trajectory and point cloud.

Congratulations! You are ready to [run UltimateSLAM in real-time](Run-Live-Demo.md).