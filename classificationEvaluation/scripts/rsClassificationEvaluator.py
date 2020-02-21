#!/usr/bin/env python

import rospy

import json

import actionlib

import matplotlib.pyplot as plt

import suturo_perception_msgs.msg



class ResultActionClient:
    def __init__(self):
        self.k = 5
        self.results = []

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
            self.results.append([i.obj_class, i.confidence_class, self.k])

            #self.saveToMd(i.obj_class, i.confidence_class, "KNN - k=13")

    def saveToMd(self, objClass, objConfidence, algorithm="KNN - k=9"):
        file = open("evaluation.md", "a")
        file.write("\n| " + objClass + " | " + str(objConfidence) + " | " + algorithm + " |")
        file.close()

    def plotResult(self):
        for (obj_class, obj_confidence, obj_k) in self.results:
            plt.scatter(obj_k, obj_confidence)

        plt.show()



if __name__ == '__main__':
    try:
        resultActionClient = ResultActionClient()
        while 1:
            resultActionClient.k = input("Please insert k:")
            result = resultActionClient.sendGoal()
            resultActionClient.saveResult(result)
            resultActionClient.plotResult()

    except rospy.ROSInterruptException:
        rospy.logerr("The program was interrupted before completion.")
