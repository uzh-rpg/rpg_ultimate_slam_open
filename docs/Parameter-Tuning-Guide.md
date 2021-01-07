# Parameter Tuning Guide

UltimateSLAM provides a number of parameters than can and should be tuned for optimal tracking performance.
An exhaustive list of parameters can be obtained by running: 

```rosrun ze_vio_ceres ze_vio_ceres_node --help```

In the rest of this page, we focus on the most important parameters.

## Event frame parameters

The parameters below allow to control how event frames are generated.

### Integration parameters

- `--vio_frame_size`: number of events used to draw each event frame (default: 15000 for DAVIS240C). Use a smaller number when the expected amount of texture is small (e.g. 5000 for *shapes_6dof* dataset). 

- `--data_size_augmented_event_packet`: size of augmented event packets (which are passed into the frontend, responsible for event frame drawing), expressed in number of events. **This value should be greater than `--vio_frame_size`**.

- `--vio_do_motion_correction`: whether to enable or disable motion correction using the IMU, estimated velocity and median scene depth (default: true). We recommend to leave it enabled unless you are using a very small frame size, or for debugging purposes.

- `--noise_event_rate`: if the local event rate is less than `noise_event_rate` per frame, those events are considered to be noise, and the VIO will discard the measurements from that event frame, adding a strong "static" prior to the backend.

### Frequency of event frame generation (for events-only pipeline)

In the events-only pipeline, you can specify fully at which rate the event frames will be created with the parameters below.

**In the events + frames pipeline, the event frame rate is the same as the grayscale image framerate (i.e. ~20 Hz for DAVIS240C)**.

**The parameters below apply to the events-only pipeline.**

- `--data_use_time_interval`: specify whether events packets are created at a fixed temporal rate (true) or fixed event rate (false).

- `--data_interval_between_event_packets`: specifies the interval between two event packets. This value is interpreted either as milliseconds if `--data_use_time_interval` is true, or as number of events if false.

## Calibration parameters

- `--timeshift_cam_imu`: temporal offset (in seconds) between the IMU and the camera. Use Kalibr to estimate it.
- `--calib_filename`: filename of the YAML camera calibration file (in the `calibration` folder).

## Logging parameters

- `--log_dir`: folder in which the logging info (estimated camera poses, timing information) will be saved.
- `--vio_trace_pose`: whether or not to output the estimated poses to a file.

## Visualization parameters

- `--vio_viz_feature_tracks`: whether to show the trace of the feature tracks.