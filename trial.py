#!/usr/bin/env python
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Imu, NavSatFix
import numpy as np
import rospy
import tf
from bresenham import bresenham
from matplotlib import pyplot as plt
import matplotlib.animation as animation
from math import sin, cos, sqrt, atan2, radians


coords = np.loadtxt('gps_waypoint.txt',delimiter=' ')
lat = np.array(coords[:,0])
lon = np.array(coords[:,1])
print(lat.shape)
