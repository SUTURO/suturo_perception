#!/usr/bin/env python

import matplotlib.pyplot as plt

import actionlib
import rospy
import os.path
import suturo_perception_msgs.msg


class ResultActionClient:
    # Initializing the action client
    def __init__(self):
        self.k = 5
        self.results = []

        rospy.init_node('rsResultActionClient', anonymous=True)

        self.client = actionlib.SimpleActionClient('perception_actionserver', suturo_perception_msgs.msg.ExtractObjectInfoAction)
        self.client.wait_for_server()

    # Sending the goal to the perception server
    def sendGoal(self):

        goal = suturo_perception_msgs.msg.ExtractObjectInfoGoal()
        self.client.send_goal(goal)

        self.client.wait_for_result()
        return self.client.get_result()

    # Saving the action result in a local data structure
    def saveResult(self, result):

        for i in result.detectionData:
            rospy.loginfo("Class: " + i.obj_class + " - confidence: %f", i.confidence_class)
            self.results.append([i.obj_class, i.confidence_class, self.k])

    # Saving the result in a .md file. Appends at then end of the file, generates a new one, if it does not exist.
    def saveToMd(self, algorithm="KNN"):
        if not os.path.exists('./evaluation.md'):
            file = open("evaluation.md", "a")
            file.write("| Class | Confidence | Algorithm | K |\n")
            file.write("|-------|------------|-----------|---|")
        else:
            file = open("evaluation.md", "a")

        for (obj_class, obj_confidence, obj_k) in self.results:
            file.write("\n| " + obj_class + " | " + str(obj_confidence) + " | " + algorithm + " | " + str(obj_k) + " |")

        file.close()

    # Plots the result using matplotlib
    def plotResult(self):
        for (obj_class, obj_confidence, obj_k) in self.results:
            plt.scatter(obj_k, obj_confidence)
            print(obj_k)

        plt.show()


# Loops and triggers the perception server until a non valid string (e.g empty) is inserted.
if __name__ == '__main__':
    try:
        resultActionClient = ResultActionClient()
        while 1:
            try:
                inputK = input("Please insert k:")
            except SyntaxError:
                print("No number was entered. Stopping.")
                break;

            if inputK == None:
                break

            else:
                resultActionClient.k = inputK;
                result = resultActionClient.sendGoal()
                resultActionClient.saveResult(result)
                resultActionClient.plotResult()         # Comment out or in to disable/enable plotting and saving to .md
                resultActionClient.saveToMd()

    except rospy.ROSInterruptException:
        rospy.logerr("The program was interrupted before completion.")
