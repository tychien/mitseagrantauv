1. Going to your catkin_ws directory

  ```bash
  cd ~/catkin_ws/src
  ```

2. download this folder to your `~/catkin_ws/src`

3. build your catkin_ws

```bash
cd ~/catkin_ws
catkin_make
```

4. Run the LiDAR, make sure it has topic /velodyne_points

```bash
#connect to the Velodyne LiDAR, then...
roscore
rostopic list
#check whether it has topic /velodyne_points
```

5. Run this code

```bash
rosrun adaptive_clustering adaptive_clustering
```

6. Using Rviz to visualize the result. Try checking the topic adaptive_clustring/marker in the Rviz


