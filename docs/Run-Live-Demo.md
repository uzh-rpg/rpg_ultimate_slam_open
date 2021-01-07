# Run Live Demo

To run UltimateSLAM, you need an event camera with a hardware-synchronized inertial measurement unit.
We have successfully tested UltimateSLAM with the [DAVIS240C and DAVIS346 cameras](https://inivation.com/buy/).

## Install DAVIS ROS driver
Before doing so, make sure to follow the instructions [here](https://github.com/uzh-rpg/rpg_dvs_ros#driver-installation) to install 
any dependencies. In particular, make sure `libcaer` is installed. Then,

- Build the DAVIS ROS driver:

```catkin build davis_ros_driver```

- An update of the udev rules is needed to run the driver:

```
source ~/uslam_ws/devel/setup.bash
roscd libcaer_catkin
sudo ./install.sh
```

More detailed instructions are available [here](https://github.com/uzh-rpg/rpg_dvs_ros) in case you are having troubles during this step.

## Calibrate your DAVIS camera

Before running UltimateSLAM, you will need to calibrate your DAVIS sensor, i.e. estimate the camera intrinsic parameters, the camera-to-IMU extrinsic parameters, as well as the time offset between the IMU and the events. Some guidance is provided in [this page](Camera-Calibration.md).

**Note that an accurate camera calibration is absolutely essential to get good tracking results. Make sure you treat this step with particular care**.

Once you have performed calibration, copy your calibration file in the right format into the `calibration` folder.

## Run UltimateSLAM live demo with the DAVIS

To run UltimateSLAM live with a DAVIS sensor, you will need to open multiple terminals at the same time. We recommend using [Terminator](https://gnometerminator.blogspot.com/p/introduction.html), which allows to easily have multiple terminal windows open at the same time and easily navigate between those windows.

Open three terminals, and launch the following commands.

#### Terminal 1

Start a `roscore`:

```roscore```

#### Terminal 2

Start the DAVIS driver:

```rosrun davis_ros_driver davis_ros_driver```

#### Terminal 3

Start UltimateSLAM:

##### Events + Frames

```roslaunch ze_vio_ceres live_DAVIS240C.launch camera_name:=<your_camera_calibration_filename> timeshift_cam_imu:=0.0028100209382249794```

##### Events only

```roslaunch ze_vio_ceres live_DAVIS240C_events_only.launch camera_name:=<your_camera_calibration_filename> timeshift_cam_imu:=0.0028100209382249794```

The calibration file will be searched for in the `calibration/` folder.
In both cases, a new RVIZ window should pop up, showing the current camera position, trajectory and current estimated point cloud.

##### Parameters

- `camera_name`: replace with the name of your calibration file in the `calibration` folder, without the `.yaml` extension.
- `timeshift_cam_imu`: temporal offset between the camera and IMU. Please replace the value above with the one estimated by Kalibr during the calibration step.
- `frame_size`: number of events to integrate in each event frame
- `motion_correction`: if 1, will perform motion correction. Otherwise, not.
- there are additional parameters in the launch files, please refer to the [parameter tuning guide](Parameter-Tuning-Guide.md) for more details.

## Important advice

- Initialization is fairly sensitive in UltimateSLAM. For best results, we advise to start with the sensor static (e.g. resting on a table). After launching UltimateSLAM, perform a few seconds of translational motion (moving left-right, up-down) with the sensor to get a good initial estimate of the IMU biases. Avoid initializing UltimateSLAM with the sensor already moving. Avoid initializing while doing mostly rotational motion with the camera.
- As with any visual-inertial odometry system, a good calibration is of paramount importance to get good tracking results.
- Proper [parameter tuning](Parameter-Tuning-Guide.md) is necessary to obtain the best results. By default, the parameters are set to work well with the DAVIS240C (240x180) sensor, and a moderate amount of texture in the scene.