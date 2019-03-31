"""
Frenet optimal trajectory generator

"""
import random
import numpy as np
import matplotlib.pyplot as plt
import copy
import math
import cubic_spline_planner
import time
import rospy
import tf
from sensor_msgs.msg import Imu, NavSatFix
from rnd.msg import Car_Feedback
from rnd.msg import Car_Input
from rnd.msg import ToPlanner

#################################################################

# Parameter
MAX_SPEED = 15.0 / 3.6  # maximum speed [m/s]
MAX_ACCEL = 2.0  # maximum acceleration [m/ss]
MAX_CURVATURE = 2.0  # maximum curvature [1/m]
MAX_ROAD_WIDTH = 7.0  # maximum road width [m]
D_ROAD_W = 1.0  # road width sampling length [m]
DT = 0.5  # time tick [s]
MAXT = 5.0  # max prediction time [m]
MINT = 4.0  # min prediction time [m]
TARGET_SPEED = 15.0 / 3.6  # target speed [m/s]
D_T_S = 5.0 / 3.6  # target speed sampling length [m/s]
N_S_SAMPLE = 1  # sampling number of target speed
ROBOT_RADIUS = 2.0  # robot radius [m]
THRESH_DIST=0.30
# cost weights
KJ = 0.1
KT = 0.1
KD = 2.0
KLAT = 10.0
KLON = 1.0

show_animation = True


class quintic_polynomial:

    def __init__(self, xs, vxs, axs, xe, vxe, axe, T):

        # calc coefficient of quintic polynomial
        self.xs = xs
        self.vxs = vxs
        self.axs = axs
        self.xe = xe
        self.vxe = vxe
        self.axe = axe

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[T**3, T**4, T**5],
                      [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
                      [6 * T, 12 * T ** 2, 20 * T ** 3]])
        b = np.array([xe - self.a0 - self.a1 * T - self.a2 * T**2,
                      vxe - self.a1 - 2 * self.a2 * T,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t**2 + \
            self.a3 * t**3 + self.a4 * t**4 + self.a5 * t**5

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
            3 * self.a3 * t**2 + 4 * self.a4 * t**3 + 5 * self.a5 * t**4

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2 + 20 * self.a5 * t**3

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t**2

        return xt


class quartic_polynomial:

    def __init__(self, xs, vxs, axs, vxe, axe, T):

        # calc coefficient of quintic polynomial
        self.xs = xs
        self.vxs = vxs
        self.axs = axs
        self.vxe = vxe
        self.axe = axe

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * T ** 2, 4 * T ** 3],
                      [6 * T, 12 * T ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * T,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t**2 + \
            self.a3 * t**3 + self.a4 * t**4

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
            3 * self.a3 * t**2 + 4 * self.a4 * t**3

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t

        return xt


class Frenet_path:

    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []


def calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0):

    frenet_paths = []

    # generate path to each offset goal
    for di in np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W):

        # Lateral motion planning
        for Ti in np.arange(MINT, MAXT, DT):
            fp = Frenet_path()

            lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

            fp.t = [t for t in np.arange(0.0, Ti, DT)]
            fp.d = [lat_qp.calc_point(t) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            # Loongitudinal motion planning (Velocity keeping)
            for tv in np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE, TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S):
                tfp = copy.deepcopy(fp)
                lon_qp = quartic_polynomial(s0, c_speed, 0.0, tv, 0.0, Ti)

                tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                # square of diff from target speed
                ds = (TARGET_SPEED - tfp.s_d[-1])**2

                tfp.cd = KJ * Jp + KT * Ti + KD * tfp.d[-1]**2
                tfp.cv = KJ * Js + KT * Ti + KD * ds
                tfp.cf = KLAT * tfp.cd + KLON * tfp.cv

                frenet_paths.append(tfp)

    return frenet_paths


def calc_global_paths(fplist, csp):

    for fp in fplist:

        # calc global positions
        for i in range(len(fp.s)):
            ix, iy = csp.calc_position(fp.s[i])
            if ix is None:
                break
            iyaw = csp.calc_yaw(fp.s[i])
            di = fp.d[i]
            fx = ix + di * math.cos(iyaw + math.pi / 2.0)
            fy = iy + di * math.sin(iyaw + math.pi / 2.0)
            fp.x.append(fx)
            fp.y.append(fy)

        # calc yaw and ds
        for i in range(len(fp.x) - 1):
            dx = fp.x[i + 1] - fp.x[i]
            dy = fp.y[i + 1] - fp.y[i]
            fp.yaw.append(math.atan2(dy, dx))
            fp.ds.append(math.sqrt(dx**2 + dy**2))

        fp.yaw.append(fp.yaw[-1])
        fp.ds.append(fp.ds[-1])

        # calc curvature
        for i in range(len(fp.yaw) - 1):
            fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

    return fplist


def check_collision(fp, ob):

    for i in range(len(ob[:, 0])):
        d = [((ix - ob[i, 0])**2 + (iy - ob[i, 1])**2)
             for (ix, iy) in zip(fp.x, fp.y)]

        collision = any([di <= ROBOT_RADIUS**2 for di in d])

        if collision:
            return False

    return True


def check_paths(fplist, ob):

    okind = []
    for i in range(len(fplist)):
        if any([v > MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
            continue
        elif any([abs(a) > MAX_ACCEL for a in fplist[i].s_dd]):  # Max accel check
            continue
        elif any([abs(c) > MAX_CURVATURE for c in fplist[i].c]):  # Max curvature check
            continue
        elif not check_collision(fplist[i], ob):
            continue

        okind.append(i)

    return [fplist[i] for i in okind]


def frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob):

    fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0)
    fplist = calc_global_paths(fplist, csp)
    fplist = check_paths(fplist, ob)

    # find minimum cost path
    mincost = float("inf")
    bestpath = None
    for fp in fplist:
        if mincost >= fp.cf:
            mincost = fp.cf
            bestpath = fp

    return bestpath


def generate_target_course(x, y):
    csp = cubic_spline_planner.Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, csp

#######################################################################################

def get_gps_waypoints(wx,wy):
    f = open("gps_waypoint.txt","r")
    lines=f.readlines()
    for x in lines:
        wx.append(float(x.split(' ')[0].strip()))
        wy.append(float(x.split(' ')[1].strip()))
    f.close()

def gps_to_map(lat,lon,ptx,pty):
    for x in range(1,len(lon)):
        dy = (lon[x]-lon[0])*40000000*math.cos((lat[0]+lat[x])*math.pi/360)/360
        dx = (lat[0]-lat[x])*40000000/360
        ptx.append(dx)
        pty.append(dy)  
    return ptx, pty

class Info(object):
    def __init__(self):
        self.CurrGPS_lat = float(-1)
        self.CurrGPS_lon = float(-1)
        self.CurrentVelocity = float(-1)
        self.Target_Velocity = float(-1)
        self.ImuYaw = float(-1)
        self.Target_Theta = float(-1)
        self.CommandMessage = Car_Input()
        self.gob = np.array([])
        self.ob = np.array([])
        self.gobx = np.array([])
        self.goby = np.array([])
        

        # Subscribers
        rospy.Subscriber("/imu/data", Imu, self.FeedbackCallbackImu)
        rospy.Subscriber("/ublox_gps/fix",NavSatFix,self.FeedbackCallbackGPS)
        rospy.Subscriber("/ToPlanner",ToPlanner,self.FeedbackCallbackObs)

    def FeedbackCallbackGPS(self,msg):
        self.CurrGPS_lat = msg.latitude
        self.CurrGPS_lon = msg.longitude

    def FeedbackCallbackImu(self,msg):
        quaternion = (msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.ImuYaw = euler[2]

    def FeedbackCallbackObs(self,msg):
        self.gobx = msg.Obstacles_x
        self.goby = msg.Obstacles_y
        np.append(self.gobx,60)
        np.append(self.goby,60)
        self.gob = np.column_stack((gobx, goby))
    
    def init(self):
        return self.CurrGPS_lat,self.CurrGPS_lon,self.ImuYaw,self.gobx,self.goby,self.gob

    def talker(self,Target_Velocity,Target_Theta):
        self.pub = rospy.Publisher('PlannerOut', Car_Input, queue_size=10)
        self.rate = rospy.Rate(5) # 10hz
        self.CommandMessage.Target_Velocity = Target_Velocity
        self.CommandMessage.Target_Steering = Target_Theta
        self.pub.publish(self.CommandMessage)
        self.rate.sleep()

##################

def get_transalation(curr_gps_lat,curr_gps_lon,ox,oy):
    curr_posy=(float(curr_gps_lon)-oy)*40000000*math.cos((float(curr_gps_lat)+ox)*math.pi/360)/360
    curr_posx=(ox-float(curr_gps_lat))*40000000/360
    return curr_posx, curr_posy

def get_transformation(pt,curr_yaw,T):  
    c, s = np.cos(curr_yaw), np.sin(curr_yaw)
    R = (np.array(((c,-s), (s, c))))  
    pt=pt.dot(R)+T
    return pt

def get_arc_length(tx,ty,st):
    arc_length=0
    for x in range(1,st):
        arc_length=arc_length+(np.hypot((tx[x-1]-tx[x]),(ty[x-1]-ty[x])))
    return arc_length

def get_lateral_dist(tx,ty,curr_posx,curr_posy):
    dist=[]
    for x in range(0,len(tx)-1):
        dist.append(np.hypot((float(curr_posx)-tx[x]),(float(curr_posy)-ty[x])))
    lat_dist=min(dist)
    st=dist.index(min(dist))
    theta1=math.atan2((ty[st]-ty[st-1]),(tx[st]-tx[st-1]))
    theta2=math.atan2((curr_posy-ty[st-1]),(curr_posx-tx[st-1]))
    if lat_dist<THRESH_DIST:
        lat_dist=0
        curr_posx=tx[st]
        curr_posy=ty[st]
    if theta2<theta1:
        lat_dist=-lat_dist
    # print(lat_dist)
    return st, lat_dist, curr_posx, curr_posy

##########################################################################################

if __name__ == '__main__':
    wx = []
    wy = []
    ptx= []
    pty = []
    get_gps_waypoints(wx,wy)
    ptx, pty = gps_to_map(wx, wy, ptx,pty)

    tx, ty, tyaw, tc, csp = generate_target_course(ptx, pty)
    # tx = list(np.asarray(tx) - 5)
    # ty = list(np.asarray(ty) - 5)
    c_d_dd = 0
    c_d_d = 0 
    area = 20.0  # animation area length [m]
    start=time.time()

    rospy.init_node("PlannerOut", anonymous=True)
    my_node = Info()
    while not rospy.is_shutdown():
        CurrGPS_lat,CurrGPS_lon,ImuYaw,gobx,goby,gob =  my_node.init()
        ob = []
        if(CurrGPS_lat!=-1 and CurrGPS_lon!=-1 and ImuYaw!=-1):
            if(len(gob)==0):
                ob = [[-20,-20]]
            else:
                ob=gob
            curr_posx, curr_posy =  get_transalation(CurrGPS_lat, CurrGPS_lon, wx[0], wy[0])
            T = [curr_posx, curr_posy]
            c_speed = 1.0
            c_acc = 1.0
            curr_yaw = ImuYaw + math.pi/2
            spt, c_d, curr_posx, curr_posy = get_lateral_dist(tx,ty,curr_posx,curr_posy)
            # c_d_d = c_speed*math.cos(math.atan2((ty[spt]-curr_posy),(tx[spt]-curr_posx))+curr_yaw)
            # c_d_dd = c_acc*math.cos(math.atan2((ty[spt]-curr_posy),(tx[spt]-curr_posx))+curr_yaw)

            s0=get_arc_length(tx,ty,spt)
            for x in xrange(0,len(ob)-1):
                ob[x,:]= .2*ob[x,:]
                ob[x,:]=get_transformation(ob[x,:],-curr_yaw,T)

            try:
                # print(CurrGPS_lat,CurrGPS_lon,ImuYaw,gobx,goby,gob)
                path = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob)
                c_d_d = path.d_d[1]
                c_d_dd = path.d_dd[1]
                if np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 1.0:
                    print("Goal")
                    break
                if show_animation:
                    plt.cla()
                    plt.plot(tx, ty)
                    plt.plot(ob[:, 0], ob[:, 1], "xk")
                    plt.plot(path.x[1:], path.y[1:], "-or")
                    plt.plot(path.x[1], path.y[1], "vc")
                    plt.xlim(path.x[1] - area, path.x[1] + area)
                    plt.ylim(path.y[1] - area, path.y[1] + area)
                    plt.arrow(curr_posx, curr_posy, math.cos(curr_yaw), math.sin(curr_yaw))
                    plt.title("v[km/h]:" + str(c_speed * 3.6)[0:4])
                    plt.grid(True)
                    plt.pause(0.0001)
            except:
                print("no pointcloud recieved")
            
            try:
                
                temp1=path.yaw[1]     
                temp2=curr_yaw 
                
                if temp1<0:
                    temp1=6.28+temp1
                if temp2<0:
                    temp2=6.28+temp2

                val = temp1-temp2
                
                if val > 3.14:
                    val = val - 6.28
                if val < -3.14:
                    val = val + 6.28
                
                val = math.degrees(val)
                
                if val > 20:
                    val = 20
                if val < -20:
                    val = -20
                
                my_node.talker(path.s_d[1],-1*val)
            except:                
                pass

    print("Finish")
    end=time.time()
    print("total time: ", end-start)

    if show_animation:
        plt.grid(True)
        plt.pause(0.0001)
        plt.show()