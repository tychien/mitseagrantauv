# PCL DataProcessing 

![rqt_graph](https://github.com/tychien/mitseagrantauv/blob/master/pcl/src/dataprocess/src/rosgraph.png)

### Receive pcl data from /velodyne_points and then filtering
[passthrough_filter](https://github.com/tychien/mitseagrantauv/blob/master/pcl/src/dataprocess/src/filter.cpp)

### After filtering, downsample the pcl data by voxel_grid method
[voxel_grid_downsample](https://github.com/tychien/mitseagrantauv/blob/master/pcl/src/dataprocess/src/downsampling.cpp)

### After downsampling, remove the outliers 
[outliers_removal](https://github.com/tychien/mitseagrantauv/blob/master/pcl/src/dataprocess/src/remove_outliers.cpp)

### After removing, cluster! (not showing anything in rostopic echo /clustered)
[cluster](https://github.com/tychien/mitseagrantauv/blob/master/pcl/src/dataprocess/src/cluster.cpp)
