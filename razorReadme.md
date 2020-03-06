# Razor IMU M0 for AUV_LAB

This document is written by Tim Chien. tychien@mit.edu

Here's the [WIKI](http://wiki.ros.org/razor_imu_9dof)

The Board 14001 [Specs](https://www.sparkfun.com/products/14001) 

## Installation & Preparation

1.  Download the [ROS](http://wiki.ros.org/melodic/Installation/Ubuntu)
1.  Download the [ArduinoIDE](https://www.arduino.cc/en/guide/linux#toc2)
    1.  and select Razor IMU M0 Board. (url) 
1.  Download [ProcessingIDE](https://processing.org/download/)
    1. Download [EJML](url) library to the Processing sketch book.
1.  Download [razor_imu_9dof](https://github.com/KristofRobot/razor_imu_9dof) to the catkin_ws/src directory. 
```bash 
git clone https://github.com/KristofRobot/razor_imu_9dof.git 
```
1.  Download this python file [convert_to_rpy.py](https://github.com/tychien/mitseagrantauv), we will use it later. 
1.  Plug in the board into the computer port, make sure to give the permission to the board using
```bash 
sudo chmod 666 ttyACM0 
```
## Calibration

1. Launch Arduino IDE and open the Serial Monitor.



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
