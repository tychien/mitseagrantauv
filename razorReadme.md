# Razor IMU M0 for AUV_LAB

This document is written by Tim Chien. tychien@mit.edu

Here's the [WIKI](http://wiki.ros.org/razor_imu_9dof)

The Board 14001 [Specs](https://www.sparkfun.com/products/14001) 

## Installation & Preparation

*  Download the [ROS](http://wiki.ros.org/melodic/Installation/Ubuntu)
*  Download the [ArduinoIDE](https://www.arduino.cc/en/guide/linux#toc2)
    *  and select Razor IMU M0 Board. (url) 
*  Download [ProcessingIDE](https://processing.org/download/)
    * Download [EJML](url) library to the Processing sketch book.
*  Download [razor_imu_9dof](https://github.com/KristofRobot/razor_imu_9dof) to the catkin_ws/src directory. 
```bash 
cd ~/catkin_ws/src (path to your catkin_ws directory)
git clone https://github.com/KristofRobot/razor_imu_9dof.git 
```
*  Download this python file [convert_to_rpy.py](https://github.com/tychien/mitseagrantauv), we will use it later. 
*  Plug in the board into the computer port, make sure to give the permission to the board using
```bash 
sudo chmod 666 ttyACM0 
```
## Calibration

1.  Launch Arduino IDE and open the Serial Monitor.
1.  Type #oc in the Seriel Monitor Command Line to enter the accelerometer calibration mode.
1.  Take the board and point straight down with x, y, z axes and do the same thing in oppisite side to get the MAX and min calibration value and write it down.
1.  Type #on in the Seriel Monitor Command Line to enter the magnitude calibration mode and do the same thing as 3.
1.  Type #on again to get into Gyro calibraion mode, and do the same thing as 3.
1.  Quit the Serial Monitor and Launch the Processing IDE. 

In 
Use the package manager [pip](https://pip.pypa.io/en/stable/) to install foobar.

```bash
pip install foobar
```

## Usage

```python
import foobar

foobar.pluralize('word') # returns 'words'
foobar.pluralize('goose') # returns 'geese'
foobar.singularize('phenomena') # returns 'phenomenon'
```

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License
[MIT](https://choosealicense.com/licenses/mit/)
