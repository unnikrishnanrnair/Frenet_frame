import rospy
import tf
from geometry_msgs.msg import Pose , PoseStamped
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import time
from math import sin, cos, sqrt, atan2, radians
import sys

# file_name = str(sys.argv[1])

lat_new = 0
lon_new = 0
lat_old = 0
lon_old = 0
i = 0
ori = 0
waypoint = []

if len(sys.argv) != 2:
    print("Please give the distance thershold")
    exit(1)

ther = int(sys.argv[1])



def ublox_gps(U):
	global lat_new, lon_new, lat_old, lon_old, i, ori
	x = y = 0
	
	lat_new = U.latitude
	lon_new = U.longitude

	# print(len(waypoint))

	if i<1:
		lat_old = lat_new
		lon_old = lon_new
		point = [lat_new,lon_new]
		waypoint.append(point)
		i=i+1
	else:

		R = 6373.0

		lat1 = radians(lat_new)
		lon1 = radians(lon_new)
		lat2 = radians(lat_old)
		lon2 = radians(lon_old)

		dlon = lon2 - lon1
		dlat = lat2 - lat1

		a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
		c = 2 * atan2(sqrt(a), sqrt(1 - a))

		distance =  R * c * 1000
		print(distance)

		# x = cos(lat2)*sin(lon2-lon1)
		# y = cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1)
		# ori = atan2(x,y)
		# if ori>-3.14 and ori<-1.57:
		# 	ori = -ori - 3*1.57
		# else:
		# 	ori = 1.57 - ori
		# print(ori)

		if distance > ther:
			point = (lat_new,lon_new)
			waypoint.append(point)
			np.savetxt('gps_waypoint.txt', waypoint)
			lat_old = lat_new
			lon_old = lon_new

def imu_orient(I):
	global ori
	
	quaternion = (
	    I.orientation.x,
	    I.orientation.y,
	    I.orientation.z,
	    I.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)

	imuyaw = euler[2]
	# print(imuyaw)


if __name__ == '__main__':

	rospy.init_node("waypoint")

	# rospy.Subscriber("/imu/data", Imu , imu_orient)
		
	rospy.Subscriber("/ublox_gps/fix", NavSatFix, ublox_gps)

	plt.show()
	rospy.spin()