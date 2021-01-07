# Camera Calibration

This page gives some guidance to perform camera calibration, i.e. to estimate your event camera's intrinsic parameters, as well as the extrinsic calibration between the event camera and the IMU.

### DAVIS calibration

Because the DAVIS sensor provides grayscale images (`/dvs/image_raw`) on the exact same sensor array as the events, it is possible to use a standard calibration tool on the grayscale images to calibrate the camera.
We suggest to use the [Kalibr toolbox](https://github.com/ethz-asl/kalibr).

For best results, we advise to record two separate calibration datasets: one to perform intrinsic parameter calibration, and the second to perform camera-to-IMU calibration. While recording one dataset to do both is sufficient in theory, the results are usually much better when recording two separate datasets.
 
#### Intrinsic parameter calibration

To perform intrinsic camera calibration, record a rosbag with about one minute of grayscale image data (`/dvs/image_raw`) with the camera looking at a [known calibration pattern](https://github.com/ethz-asl/kalibr/wiki/calibration-targets) (we suggest to use an Aprilgrid) from different angles. Moving the camera with rather slow motion would be preferred there to minimize motion blur in the images. Also make sure to cover as many angles as possible to get good results.

```
# Record rosbag
rosbag record -O cam_calib.bag /dvs/image_raw
# Then, estimate camera intrinsics with Kalibr
kalibr_calibrate_cameras --target ~/uslam_ws/src/ultimate_slam/calibration/kalibr_targets/april_5x4.yaml --bag cam_calib.bag --models pinhole-radtan --topics /dvs/image_raw --show-extraction
```

#### Camera-to-IMU calibration

To perform camera-to-IMU calibration, record a rosbag with about one minute of grayscale images + IMU data (`/dvs/image_raw` and `/dvs/imu`).
As before, the camera should always point towards the calibration pattern. Unlike above, for this dataset it is preferrable that the camera should move somewhat faster to properly excite the IMU. However, **the motion must be as smooth as possible**, i.e. avoid stops, or very jerky motions. Make sure to excite all degrees of freedom of the accelerometer (X/Y/Z) and the gyroscope (yaw, pitch, roll).

```
# Record rosbag
rosbag record -O imu_cam_calib.bag /dvs/image_raw /dvs/imu
# Then, estimate camera-to-IMU extrinsics with Kalibr
kalibr_calibrate_imu_camera --target ~/uslam_ws/src/ultimate_slam/calibration/kalibr_targets/april_5x4.yaml --bag imu_cam_calib.bag --cam camchain-cam_calib.yaml --imu ~/uslam_ws/src/ultimate_slam/calibration/imu/davis_mpu6150.yaml --time-calibration
```

#### Convert to our calibratoin format:

```
kalibr_swe_config --cam camchain-imucam-calib.yaml --mav camera --out camera.yaml
```

#### Account for the time delay between camera and IMU messages

Once you run ```kalibr_calibrate_imu_camera```, don't forget to use the following flag when running live UltimateSLAM: `timeshift_cam_imu:=0.0028133308512579796` where your replace this number by the one given by Kalibr.
