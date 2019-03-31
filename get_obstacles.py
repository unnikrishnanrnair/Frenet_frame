from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Imu, NavSatFix
import numpy as np
import rospy
from rnd.msg import ToPlanner
import tf
from matplotlib import pyplot as plt
import matplotlib.animation as animation
from math import sin, cos, sqrt, atan2, radians

count = 1
grid_size = 128
m_per_cell = 0.2
height_thresh = 0.2
occ_size = 128
scale = 5

class lidar(object):
	def __init__(self):
		self.velo = np.array([])
		self.CommandMessage = ToPlanner()
		rospy.Subscriber("/velodyne_points", PointCloud2, self.scan_callback)

	def scan_callback(self,data):
		temp=[]
		for p in pc2.read_points(data, skip_nans=True):
			temp.append([p[0],p[1],p[2]])

		self.velo = np.array(temp)

	def talk(self,obids):
		self.pub = rospy.Publisher('ToPlanner', ToPlanner, queue_size=10)
		self.rate = rospy.Rate(10) # 5hz
		self.CommandMessage.Obstacles_x = obids[0] - 64
		self.CommandMessage.Obstacles_y = obids[1] - 64
		self.pub.publish(self.CommandMessage)
		self.rate.sleep()
	
	def init(self):
		return self.velo

def pointcloud_to_xy(velo,obstacle_list):
	
	global count,grid_size,grid_offset,occ_size,m_per_cell,scale
	num_points = velo.shape[0]
	minxy = np.zeros([grid_size,grid_size])
	maxxy = np.zeros([grid_size,grid_size])
	initxy = np.zeros([grid_size,grid_size])
	occ_map = np.ones([grid_size,grid_size])

	for n in range(num_points):
		px = velo[n][0]
		py = velo[n][1]
		pz = velo[n][2]
		x = (int)(px/m_per_cell)
		y = (int)((grid_size/2)+py/m_per_cell)

		if(x>=0 and x<grid_size and y>=0 and y<grid_size):
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

	for j in range(points.shape[0]):
		x = (int)((occ_size/2) + points[j][0]*scale)
		y = (int)((occ_size/2) + points[j][1]*scale)
		if x >=0 and x<occ_size and y>=0 and y<occ_size:
			occ_map[x][y] = 0

	obids = np.where(occ_map==0)
	# plt.plot(obids[0],obids[1],'.b')
	# plt.savefig('../dataset1/'+str(count)+'.png')
	# plt.clf()
	# count = count + 1
	return obids
if __name__ == '__main__':
    
	obids = np.empty([2,2])
	velo=np.array([])
	rospy.init_node("ToPlanner", anonymous=True)
	pub_obs_node = lidar()

	while not rospy.is_shutdown():
		velo = pub_obs_node.init()
		obids = pointcloud_to_xy(velo,obids)
		pub_obs_node.talk(obids)