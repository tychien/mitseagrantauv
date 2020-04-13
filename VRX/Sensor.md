# 4 Camera on Rex in VRX 

This document is written by Tim Chien. tychien@mit.edu

# Add the Cameras in the sensor_config.yaml

with position and orientation parameters
![yaml](https://github.com/tychien/mitseagrantauv/blob/master/VRX/Screenshot%20from%202020-04-12%2016-30-23.png)

in the vrx gazebo environment
![VRX](https://github.com/tychien/mitseagrantauv/blob/master/VRX/Screenshot%20from%202020-04-12%2017-13-32.png)

how it looks like on the REX 
![REX](https://github.com/tychien/mitseagrantauv/blob/master/VRX/Screenshot%20from%202020-04-12%2016-29-04.png)

View from the cameras in the Rviz 
![Rviz](https://github.com/tychien/mitseagrantauv/blob/master/VRX/Screenshot%20from%202020-04-12%2017-15-16.png)


Infrared
| IR 25 18 IM 15 8 | IR 25 4 IM 15 16 | IR 30 12 IM 15 22.5 |
:-------------------------:|:-------------------------:|:-------------------------:
both IR cameras have 25 degrees difference from normal, and the distance between left IR and right IR is 18 inches;both Image Cameras have 15 degrees difference from normal, and left camera is 8 inches away from right camera| both IR cameras have 25 degrees difference from normal, and the distance between left IR and right IR is 4 inches; both Image Cameras have 15 degrees difference from normal, and left camera is 16 inches away from right camera | both IR cameras have 30 degrees difference from normal, and the distance between left IR and right IR is 12 inches; both Image Cameras have 15 degrees difference from normal, and left camera is 22.5 inches away from right camera
![IR 25 18 IM 15 8](https://github.com/tychien/mitseagrantauv/blob/master/CAMERA_ANGLE/CameraAngel/CameraAngel.009.jpeg) | ![IR 25 10 IM 15 16](https://github.com/tychien/mitseagrantauv/blob/master/CAMERA_ANGLE/CameraAngel/CameraAngel.006.jpeg) | ![IR 30 6 IM 15 22.5](https://github.com/tychien/mitseagrantauv/blob/master/CAMERA_ANGLE/CameraAngel/CameraAngel.003.jpeg)

From the cases above, we can know that with smaller distance between the two cameras, the lesser distortion it get. We have much normal view from the InfraRed Camer at a distance of 4 inches with 25 degrees of angle. The TOTAL Infrared Camera ANGLE is 75+(25x2)=125 degrees ; we have better Image Camera view at a distance of 8 inches with 15 degrees of angle. The TOTAL Image Camera ANGLE is 48+(15x2)=78 degrees.