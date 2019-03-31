from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Imu, NavSatFix
import numpy as np
import rospy
from rnd.msg import ToPlanner
import tf
from bresenham import bresenham
from matplotlib import pyplot as plt
import matplotlib.animation as animation
from math import sin, cos, sqrt, atan2, radians


# fig = plt.figure()
# ax1 = fig.add_subplot(1,1,1)

count = 1
grid_size = 128
m_per_cell = 0.2
height_thresh = 0.2
occ_size = 128
scale = 5
imuyaw = 0
occ_goal = []
tolerence = 12
j = 0
obids = np.empty([2,2])

lidar_count = 0
gps_count = 0



def scan_callback(data):
	global count,m_per_cell,height_thresh,grid_size,obids,unids,occ_goal,scale,lidar_count
	velo = []

	# print('lidar_count = ',lidar_count)
	# lidar_count += 1
	
	for p in pc2.read_points(data, skip_nans=True):
		velo.append([p[0],p[1],p[2]])

	velo = np.array(velo)
	# print(velo.shape)
	num_points = velo.shape[0]

	minxy = np.zeros([grid_size,grid_size])
	maxxy = np.zeros([grid_size,grid_size])
	initxy = np.zeros([grid_size,grid_size])
	occ_map = np.ones([grid_size,grid_size])
	ispoint = np.zeros([grid_size,grid_size])
	seen_map = np.zeros([grid_size,grid_size])

	for n in range(num_points):
		px = velo[n][0]
		py = velo[n][1]
		pz = velo[n][2]
		x = (int)((grid_size/2)+px/m_per_cell)
		y = (int)((grid_size/2)+py/m_per_cell)
		# print(x,y)
		if(x>=0 and x<grid_size and y>=0 and y<grid_size):
			if not ispoint[x][y]:
				ispoint[x][y] = 1
			if pz < 0.1:
				if not initxy[x][y]:
					initxy[x][y] = 1						
					minxy[x][y]=pz
					maxxy[x][y]=pz
				else:
					minxy[x][y] = min(minxy[x][y],pz)
					maxxy[x][y] = max(maxxy[x][y],pz)

	grid_offset = grid_size/2*m_per_cell			
	f = 0
	points = np.array([])
	un_points = np.array([])

	for x in range(grid_size):
		for y in range(grid_size):
			if maxxy[x][y] - minxy[x][y] > height_thresh:
				px = -grid_offset + (x*m_per_cell)
				py = -grid_offset + (y*m_per_cell)	

				if f == 0:
					points = np.array([[px,py]])
					f = 1
				else:
					points = np.append(points,[[px,py]],axis=0)		

			if ispoint[x][y] == 1:	
				path = list(bresenham(int(grid_size/2),int(grid_size/2),x,y))
				for o,p in path:
					if o >=0 and o<grid_size and p>=0 and p<grid_size:
						seen_map[o][p] = 1

	initxy[seen_map==1] = 1
	inv_init = np.logical_not(initxy)
	un_points = np.transpose(np.nonzero(inv_init))
	un_points = -grid_offset + (un_points*m_per_cell)


	# for j in range(un_points.shape[0]):
	# 	x = (int)((occ_size/2) + un_points[j][0]*scale)
	# 	y = (int)((occ_size/2) + un_points[j][1]*scale)
	# 	if x >=0 and x<occ_size and y>=0 and y<occ_size:					
	# 		occ_map[x][y] = -1

	for j in range(points.shape[0]):
		x = (int)((occ_size/2) + points[j][0]*scale)
		y = (int)((occ_size/2) + points[j][1]*scale)
		if x >=0 and x<occ_size and y>=0 and y<occ_size:
			occ_map[x][y] = 0

	if len(occ_goal) != 0:
		for t in range(len(occ_goal)):
			gx = occ_goal[t][0]
			gy = occ_goal[t][1]
			occ_map[gx][gy] = 2			


	obids = np.where(occ_map==0)

	# plt.plot([64],[64],'.k') 
	# plt.plot(obids[0],obids[1],'.b')
	# plt.axis((0,128,0,128))

	# plt.savefig('../dataset1/'+str(count)+'.png')

	# plt.clf()
	# count += 1



def talk():
	global obids
	CommandMessage = ToPlanner()
	pub = rospy.Publisher('ToPlanner', ToPlanner, queue_size=10)
	rate = rospy.Rate(2) # 5hz
	while not rospy.is_shutdown():
		CommandMessage.Obstacles_x = obids[0] - 64
		CommandMessage.Obstacles_y = obids[1] - 64
		# print(len(updated))
		# print(len(obids[1]))
		pub.publish(CommandMessage)
		rate.sleep()
				
#============================================================================


#=============================================================================

if __name__ == '__main__':
    
	# rospy.init_node("occ_map", anonymous=True)


	rospy.Subscriber("/velodyne_points", PointCloud2, scan_callback)

	talk()

	# plt.show()

	rospy.spin()