#!/usr/bin/env python
import rospy
import json
from std_msgs.msg import String
from robosherlock_msgs.msg import RSObjectDescriptions
from perception_msgs.msg import RSObject

def callback(data):    
    pub = rospy.Publisher('/perception_output/planning', RSObject, queue_size=10)

    for value in data.obj_descriptions:
        # parsing as dict and getting content of PoseAnnotation annotator
        jsonDir = json.loads(value);

        poseAnnotation = jsonDir['rs.annotation.PoseAnnotation']
        geometryAnnotation = jsonDir['rs.annotation.Geometry']
        colorName = jsonDir['rs.annotation.SemanticColor'][0]['color']

        poses = poseAnnotation[0]['world']['rs.tf.StampedPose']['translation']
        poseAnnotationX = poses[0]
        poseAnnotationY = poses[1]
        poseAnnotationZ = poses[2]

	geometry = geometryAnnotation[0]['boundingBox']['rs.pcl.BoundingBox3D']

        obj = RSObject()
        obj.x = poseAnnotationX
        obj.y = poseAnnotationY
        obj.z = poseAnnotationZ
	obj.w = geometry['width']
        obj.d = geometry['depth']
        obj.h = geometry['height']
        obj.confidence = 1
        obj.color_name = colorName;
        
        outputStr = colorName + " object at " + str(poseAnnotationX) + " " + str(poseAnnotationY) + " " + str(poseAnnotationZ) + " w=" + str(obj.w) + " h=" + str(obj.h) + " d=" + str(obj.d) + "\n"
        rospy.loginfo(outputStr)
        
        pub.publish(obj)
    
def listener():

    rospy.init_node('hermes_planning', anonymous=True)

    rospy.Subscriber("/perception_pipeline/result_advertiser", RSObjectDescriptions, callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
