# BLASER_ROS

This ROS package contains software for hand-held Blaser, including

* Laser plane calibration
* Laser points detection and triangulation
* Blaser resolution (sensitivity) analyzation
* Laser point cloud coloring (RGB value estimation), which is also implemented
  in the SLAM package (**blaser_slam**)

The library **camera_model** (forked from **camodocal**) are extensively in this
package.

There are two main work flows:

1. Running SLAM (involves calibration & running triangulation)
2. Performing sensitivity evaluation

## 1. Running SLAM

### 1.0 Pre-check

Make sure you've got Blaser's driver running and you have three ROS topics:

1. Visual frames (sensor_msgs/Image, image stream observing the environment with
   the laser turned off, at least 20 hz)
2. Profiling frames (sensor_msgs/Image, image stream observing the laser stripe,
   at least 20 hz)
3. IMU data (sensor_msgs/Imu, at least 100 hz)

Visualize these topics (use rqt_image_view and rqt_plot) to check if they are
valid.

### 1.1 Sensor calibration

There are three calibration tasks: camera intrinsics, camera-IMU extrinsics, and
camera-laser extrinsics.

#### Camera intrinsics

**First collect images observing a checkerboard from various angles.**
It is a good idea to use ROS tool **camera_calibration** to collect images, e.g.

`rosrun camera_calibration cameracalibrator.py --size 10x7 --square 0.005 image:
=/image_topic'

1. Follow its instructions to collect images to satisfy all its score bars
2. Click calibrate, and wait until it finishes
3. Click save to save the images and the calibration results as a tarball. The
   terminal will say the tarball's location, usually in `/tmp/`.
4. Create a folder for this data in `blaser_ros/calib_data` and uncompress all
   the images into it. Throw out the calibration result since we only need the
   images.

**Secondly, use package `camera_model` to perform calibration.**
`rosrun camera_model Calibration -w 7 -h 10 -s 5 -i ./ -v --camera-model mei`

Use help (`rosrun camera_model Calibration --help`) to see all the parameters.

Be careful about what camera model you use. For a narrow FoV camera without huge
distortion, use `pinhole` model should suffice. If you use a large FoV camera
with a lot of distortion, use `mei` or `kannala-brandt`. Typically you can try
all the camera models and choose the one with the smallest reprojection error.

**Finally, examine the intrinsics parameters and result**

1. Is the reprojection error reasonable? (should be at least < 1.0 px)
2. Is fx and fy similar?
3. Is cx and cy reasonable? cx ~ width / 2, cy ~ height / 2
4. Is image height and width correct?

You can also generate undistorted images. All the straight lines in real world
should appear as perfectly straight on the image.

#### Camera-IMU extrinsics

Now we have to calibrate the Euclidean transformation between the IMU frame and
the camera frame.

We use **Kalibr** to perform this calibration. It is a powerful calibration tool
that can calibrate camera intrinsics, camera-IMU extrinsics, multi-camera
extrinsics, etc. Follow its instructions for installation and usage.

A few tips:

* Use a different catkin workspace for **Kalibr**, since it uses `catkin build`.
* Since extrinsics calibrtion requires camera instrinsics, you may need to redo
  camera intrinsics calibration using Kalibr, if the camera model used in
  `camera_model` is not compatibale with Kalibr.
* You need IMU parameters (random walk and noise for accelerometer and
  gyroscope)
  for both extrinsics calibration and SLAM. `Kalibr`'s wiki has a tutorial on
  how to determine these four parameters. You can also use `imu_utils` package
  on Github to calibrate them.
* **Kalibr**'s wiki has some great tutorials on IMU modeling.

The calibration may take a long time. After it finishes, examine the rotation
and translation result. You should have a pretty good guess by the mechanical
design CAD file or by just looking at the sensor.

#### Camera-laser extrinsics

The extrinsics parameters between camera and laser is the 3D position of the
laser plane in the camera reference frame, defined as $ax + by + cz + d = 0$. In
order to determine the plane's position, we take sample points from this plane
and then try to fit a 3D plane to these 3D points. We obtain sample points from
images of a checkerboard where the laser stripe is projected onto the
checkerboard. Since the checkerboard defines a 3D plane, we can get a 3D point
position for each 2D laser point on the image, which is the itersection between
the checkerboard plane and the line-of-sight ray.

We first need to make sure that the laser stripe detection is working. The laser
stripe detector basically performs an HSV filter with five parameters hue_min,
hue_max, sat_min, val_min, and val_ratio, defining a 3D range filter H in \[hue_low,
hue_high\], S in \[sat_low, 255\], and V in 
\[max(val_low, image_max_val * val_ratio), 255\]. Note that V range is dynamic 
and is set with every image.

When using red laser and you want a hue range containing the hue=180 value,
set hue_min > hue_max and the program will generate two separate hue ranges: 
one is \[hue_min, 180\] and the other is \[0, hue_max\]. 


To set these parameters, first use `python scripts/im_saver.py [image_topic]` to
save a couple of sample images, then use `python scripts/im_hsv_viewer.py
[image_file]` to determine the HSV value of the laser stripe and set appropriate
values for the threshold parameters. Load these values in a config file (todo
give example config and dir), which will be used in the calibration later.

To test these parameters, run `laser_stripe_detector` node with sample images
and the config file. (todo turn on lsd visualization).

## Laser plane calibration

Navigate to this project's directory  
`roscd blaser_ros`

Make sure you have calibration data and config files, then run  
`rosrun blaser_ros laser_calib calib/laser_plane_calib/data/vio_640_1 config/target_config/checkerboard.yaml config/env_config/checkerboard.yaml config/cam_config/pinhole.yaml`

After the calibration successfully finished, you can visualize the result  
`python3 script/vis_laser_calib_result.py calib_result.txt`
