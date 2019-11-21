#!/usr/bin/env python
import rospy
import json
from robosherlock_msgs.msg import RSObjectDescriptions

def callback(data):
    value = data.obj_descriptions[0]
    vector = value[value.find("\"translation\":")+15:value.find("\"frame\"")-2].split(",", 3)
    
    x = float(vector[0])
    y = float(vector[1])
    z = float(vector[2])

    rospy.loginfo("Got x = " + str(x) + ", y = " + str(y) + ", z = " + str(z) + "\n")
    
def listener():

    rospy.init_node('perception_subscriber', anonymous=True)

    rospy.Subscriber("/perception_pipeline/result_advertiser", RSObjectDescriptions, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
