# Razor IMU M0 for AUV_LAB

This document is written by Tim Chien. tychien@mit.edu

Here's the Razor IMU official [WIKI](http://wiki.ros.org/razor_imu_9dof)

The Board 14001 [Specs](https://www.sparkfun.com/products/14001) 

![razor_imu_14001](https://github.com/tychien/mitseagrantauv/blob/master/IMG_7081.jpg)
## Installation & Preparation

*  Download the [ROS](http://wiki.ros.org/melodic/Installation/Ubuntu)
*  Download the [ArduinoIDE](https://www.arduino.cc/en/guide/linux#toc2)
    *  and select Razor IMU M0 Board. (url) 
*  Download [ProcessingIDE](https://processing.org/download/)
    * Download [EJML](https://github.com/tychien/mitseagrantauv/blob/master/EJML) library to the Processing sketch book.
```bash 
cd ~/sketchbook/libraries
```
*  Download [razor_imu_9dof](https://github.com/KristofRobot/razor_imu_9dof) to the `catkin_ws/src` directory. 
```bash 
cd ~/catkin_ws/src (path to your catkin_ws directory)
git clone https://github.com/KristofRobot/razor_imu_9dof.git 
```
*  Download this python file [convert_to_rpy.py](https://github.com/tychien/mitseagrantauv/blob/master/convert_to_rpy.py), we will use it later. 
*  Plug in the board into the computer port, make sure to give the permission to the board using
```bash 
sudo chmod 666 ttyACM0 
```
## Calibration

1.  Launch Arduino IDE and open the Serial Monitor.
1.  Type `#oc`in the Seriel Monitor Command Line to enter the accelerometer calibration mode.
1.  Take the board and point straight down with x, y, z axes and do the same thing in oppisite side to get the MAX and min calibration value and write it down.
1.  Type `#on`in the Seriel Monitor Command Line to enter the magnitude calibration mode and do the same thing as 3.
1.  Type `#on`again to get into Gyro calibraion mode, and do the same thing as 3.
1.  Quit the Serial Monitor and Launch the Processing IDE. 
1.  Open the Magnetometer Calibration file which is in `catkin_ws/src/razor_imu_9dof/magnetometer_calibration/Processing` directory.
1.  Run the code. It will pop-up a new window with a surphace. If not, then press `r` to restart.
![IMAGE of the pop-up window](https://github.com/tychien/mitseagrantauv/blob/master/processing_megnetometer_calibration.jpg)
1.  Move the razor imu to make the dots cover the surphace in the pop-up window. Press `Space` to show the calibration value.
1.  Write the calibration value into `my_razor.yaml` file. 
```bash 
vim ~/catkin_ws/src/razor_imu_9dof/config/my_razor.yaml
```
##  Launch the board

```bash
$ Roslaunch razor_imu_9dof razor-pub.launch 
```

Open the second terminal 

```bash
$ cd (Path of the convert_to_rpy.py)
$ python2 convert_to_rpy.py 
```

Open the third terminal 

```bash
$ rostopic echo /heading 
``` 
if it works well, it'll show the heading 
![show the heading](https://github.com/tychien/mitseagrantauv/blob/master/heading.jpg)
## Notification

I've tried to work through these steps including calibration and measuring on the metal desk for the previos times, 

and it still drifting after calibration.

Then I moved the razor imu to the wood desk and keep it far away from any electricle devices including my phone and computer. 

Then, the drifting problem has been resolved. 

It looks quite well like this...
![deg55](https://github.com/tychien/mitseagrantauv/blob/master/deg55.jpg)
![deg145](https://github.com/tychien/mitseagrantauv/blob/master/deg145.jpg)
![deg235](https://github.com/tychien/mitseagrantauv/blob/master/deg234.jpg)
![deg325](https://github.com/tychien/mitseagrantauv/blob/master/deg324.jpg) 
