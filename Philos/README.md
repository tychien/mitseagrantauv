# trouble shooting

### ImportError: No module named generator_scripts.wamv_config.configure_wamv

try to `source ~/vrx_ws/devel/setup.bash`

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
