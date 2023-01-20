#!/usr/bin/env python3
#from cgi import print_directory 
#from re import T 
#import sys
import rospy

from std_msgs.msg import Int32
from std_msgs.msg import String
rospy.init_node('listener_and_pub_to_Ard', anonymous=False)

pub = rospy.Publisher('Ultrasonic_data_sub', String, queue_size=10)
#rate = rospy.Rate(10)

def callback(data):
    num=data.data
    #rospy.loginfo("I heard %d", num)
    
    if (num<7):
        str = "Object_Detected"
        rospy.loginfo(str)
        pub.publish(str)

    else:
        str = "No Object"
        rospy.loginfo(str)
        pub.publish(str)
            

def listener():
    rospy.Subscriber("/ultrasound_data", Int32, callback)
     # 10hz
    #rospy.init_node('Pub_to_arduino', anonymous=True)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
  
    
    

if __name__ == '__main__':
    rospy.loginfo("Listener node started")
    listener()
    #talker()

