# 4 Camera on Rex in VRX 

This document is written by Tim Chien. tychien@mit.edu


#With these desired angles
![IR 25 18 IM 15 8](https://github.com/tychien/mitseagrantauv/blob/master/CAMERA_ANGLE/CameraAngel/CameraAngel.009.jpeg)

# Add the Cameras and edit the position and orientation parameters in the sensor_config.yaml file

```yaml
wamv_camera:
    - name: left_infred_camera
      x: 0.81
      y: 0.18
      z: 1.9        
      P: ${radians(5)}
      Y: 0.436      # radian 0.436 = 25 degrees
    - name: left_image_camera
      x: 0.81
      y: 0.04
      z: 1.9
      P: ${radians(5)}
      Y: 0.262      # radian 0.262 = 15 degrees 
    - name: right_image_camera
      x: 0.81
      y: -0.04
      z: 1.9
      P: ${radians(5)}
      Y: -0.262     # radian -0.262 = -15 degrees
    - name: right_infrared_camera
      x: 0.81
      y: -0.18
      z: 1.9
      P: ${radians(5)}
      Y: -0.436     # radian -0.436 = -25 degrees
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

#in the vrx gazebo environment
![VRX](https://github.com/tychien/mitseagrantauv/blob/master/VRX/Screenshot%20from%202020-04-12%2017-13-32.png)

#how it looks like on the REX 
![REX](https://github.com/tychien/mitseagrantauv/blob/master/VRX/Screenshot%20from%202020-04-12%2016-29-04.png)

#View from the cameras in the Rviz 
![Rviz](https://github.com/tychien/mitseagrantauv/blob/master/VRX/Screenshot%20from%202020-04-12%2017-15-16.png)


