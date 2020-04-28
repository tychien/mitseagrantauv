# 5 Cameras on Philos 

This document is written by Tim Chien. tychien@mit.edu

## Desired angles and positions

![desired](https://github.com/tychien/mitseagrantauv/blob/master/Philos/CameraAngel/CameraAngel.001.png)


## yaml file


```yaml

wamv_camera:
    - name: left_image_camera
      x: 0.7
      y: 0.26
      z: 1.9
      P: ${radians(5)}
      Y: ${radians(25)}      # radian 0.438 = 25 degrees
    - name: left_infrared_camera
      x: 0.7
      y: 0.07
      z: 1.9
      P: ${radians(5)}
      Y: ${radians(30)}      # radian 0.524 = 30 degrees
    - name: middle_image_camera
      x: 0.7
      y: 0
      z: 1.9
      P: ${radians(5)}
      Y: 0
    - name: right_infrared_camera
      x: 0.7
      y: -0.07
      z: 1.9
      P: ${radians(5)}
      Y: ${radians(-30)}     # radian -0.524 = -30 degrees
    - name: right_image_camera
      x: 0.7
      y: -0.26
      z: 1.9
      P: ${radians(5)}
      Y: ${radians(-25)}     # radian -0.436 = -25 degrees
wamv_gps:
    - name: gps_wamv
      x: -0.85
      z: 1.9
wamv_imu:
    - name: imu_wamv
      x: 0.85
      y: 0.0
lidar:
    - name: lidar_wamv
      type: 32_beam
      x: 0.85
      P: ${radians(1)}

```

## In the VRX_GAZEBO environment
![VRX1](https://github.com/tychien/mitseagrantauv/blob/master/Philos/CameraAngel/robotx_example_course_gzclient_camera(1)-2020-04-27T15_47_53.811145.jpg)

![VRX2](https://github.com/tychien/mitseagrantauv/blob/master/Philos/CameraAngel/robotx_example_course_gzclient_camera(1)-2020-04-27T15_48_30.769825.jpg)


## Cameras on the Philos
![Philos](https://github.com/tychien/mitseagrantauv/blob/master/Philos/CameraAngel/robotx_example_course_gzclient_camera(1)-2020-04-28T13_46_58.735613.jpg)


## View from the Rviz
![Rviz](https://github.com/tychien/mitseagrantauv/blob/master/Philos/CameraAngel/Screenshot%20from%202020-04-27%2015-50-18.png)

# trouble shooting

### ImportError: No module named generator_scripts.wamv_config.configure_wamv

Remember to `source ~/vrx_ws/devel/setup.bash`
