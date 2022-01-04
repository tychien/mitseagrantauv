import numpy as np
import open3d
import random

listPoints = [[10,20,30],[10,20,40],[10,20,50],[10,20,60]]
'''
for i in range(10000):
    x = random.random()
    y = random.random()
    z = random.random()
    listPoints.append([x,y,z])
'''
arrayPoints = np.asarray(listPoints)

point_cloud = open3d.geometry.PointCloud()
point_cloud.points = open3d.utility.Vector3dVector(arrayPoints)
open3d.visualization.draw_geometries([point_cloud])
