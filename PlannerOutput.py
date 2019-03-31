#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import Imu, NavSatFix
from rnd.msg import Car_Feedback
from rnd.msg import PlannerToCar
CurrGPS_lat = float(0)
CurrGPS_lon = float(0)
CurrentVelocity = float(0)
Target_Velocity = float(0)
ImuYaw = float(0)
Target_Theta = float(0)
CommandMessage = PlannerToCar()

def FeedbackCallbackVelocity(msg):
    global CurrentVelocity
    CurrentVelocity = msg.CurrVelocity

def FeedbackCallbackGPS(msg):
    global CurrGPS_lon,CurrGPS_lat  
    CurrGPS_lat = msg.latitude
    CurrGPS_lon = msg.longitude



def FeedbackCallbackImu(msg):
    global ImuYaw
    quaternion = (msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)

    ImuYaw = euler[2]

def listener():
    rospy.Subscriber("/Feedback", Car_Feedback, FeedbackCallbackVelocity)
    rospy.Subscriber("/imu/data", Imu, FeedbackCallbackImu)
    rospy.Subscriber("/ublox_gps/fix",NavSatFix,FeedbackCallbackGPS)
    
    
def talker():
    pub = rospy.Publisher('PlannerOut', PlannerToCar, queue_size=10)
    rate = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        CommandMessage.Target_Velocity = Target_Velocity
        CommandMessage.Target_Theta = Target_Theta
        pub.publish(CommandMessage)
        rate.sleep()

def compute():
    a = 0


if __name__ == '__main__':
    rospy.init_node('Planner', anonymous=True)
    print("LOL")
    try:
        listener()
        compute()
        talker()
    except rospy.ROSInterruptException:
        pass