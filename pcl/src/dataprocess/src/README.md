copy the mitseagrantauv/pcl ros package to your catkin repository then build 

# PCL DataProcessing 

![rqt_graph](https://github.com/tychien/mitseagrantauv/blob/master/pcl/src/dataprocess/src/rosgraph.png)

### 1. Receive pcl data from /velodyne_points and then filtering
```bash
rosrun dataprocess passthrough
```

[passthrough_filter](https://github.com/tychien/mitseagrantauv/blob/master/pcl/src/dataprocess/src/passthrough.cpp)

### 2. After filtering, downsample the pcl data by voxel_grid method
```bash
rosrun dataprocess downsampling
```

[voxel_grid_downsample](https://github.com/tychien/mitseagrantauv/blob/master/pcl/src/dataprocess/src/downsampling.cpp)

### 3. After downsampling, remove the outliers
```bash
rosrun dataprocess remove_outliers
```

[outliers_removal](https://github.com/tychien/mitseagrantauv/blob/master/pcl/src/dataprocess/src/remove_outliers.cpp)

### 4. After removing, cluster! (not showing anything in rostopic echo /clustered)
```bash
rosrun dataprocess cluster
```

[cluster](https://github.com/tychien/mitseagrantauv/blob/master/pcl/src/dataprocess/src/cluster.cpp)


### 5. Compiling 
[CMakelist](https://github.com/tychien/mitseagrantauv/blob/master/pcl/src/dataprocess/CMakeLists.txt)

| RAW | Filtered |
:-------------------------:|:-------------------------:
|![raw](https://github.com/tychien/mitseagrantauv/blob/master/pcl/src/dataprocess/src/Screenshot%20from%20raw.mp4.png)|![filtered](https://github.com/tychien/mitseagrantauv/blob/master/pcl/src/dataprocess/src/Screenshot%20from%20filter.mp4.png)
| Downsampled | Outlier_removed |
|![downsampled](https://github.com/tychien/mitseagrantauv/blob/master/pcl/src/dataprocess/src/Screenshot%20from%20downsampled.mp4.png)|![outlier_removal](https://github.com/tychien/mitseagrantauv/blob/master/pcl/src/dataprocess/src/Screenshot%20from%20outlier_removal.png)
