#!/usr/bin/env python

import rospy

import json

import actionlib

import suturo_perception_msgs.msg



class ResultActionClient:
    def __init__(self):
        rospy.init_node('rsResultActionClient', anonymous=True)

        self.client = actionlib.SimpleActionClient('perception_actionserver', suturo_perception_msgs.msg.ExtractObjectInfoAction)
        self.client.wait_for_server()

    def sendGoal(self):

        goal = suturo_perception_msgs.msg.ExtractObjectInfoGoal()
        self.client.send_goal(goal)

        self.client.wait_for_result()
        return self.client.get_result()

    def saveResult(self, result):

        for i in result.detectionData:
            rospy.loginfo("Class: " + i.obj_class + " - confidence: %f", i.confidence_class)
            self.saveToMd(i.obj_class, i.confidence_class, "KNN - k=13")

    def saveToMd(self, objClass, objConfidence, algorithm="KNN - k=9"):
        file = open("evaluation.md", "a")
        file.write("\n| " + objClass + " | " + str(objConfidence) + " | " + algorithm + " |")
        file.close()




if __name__ == '__main__':
    try:
        resultActionClient = ResultActionClient()
        result = resultActionClient.sendGoal()
        resultActionClient.saveResult(result)
    except rospy.ROSInterruptException:
        rospy.logerr("The program was interrupted before completion.")
