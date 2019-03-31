#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import Imu, NavSatFix
from rnd.msg import Car_Feedback
from rnd.msg import PlannerToCar
CurrGPS_lat = 0
CurrGPS_lon = 0
Yaw = 0
CommandMessage = PlannerToCar()

def FeedbackCallbackImu(msg):
    global Yaw
    quaternion = (msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)

    Yaw = euler[2]

def listener():
    rospy.Subscriber("/imu/data", Imu, FeedbackCallbackImu)

def talker():
    pub = rospy.Publisher('PlannerOut', PlannerToCar, queue_size=10)
    rate = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        CommandMessage.Target_Velocity = 0
        CommandMessage.Target_Theta = Yaw
        pub.publish(CommandMessage)
        rate.sleep()



if __name__ == '__main__':
    rospy.init_node('Planner', anonymous=True)
    print("LOL")
    try:
        listener()
        talker()
    except rospy.ROSInterruptException:
        pass