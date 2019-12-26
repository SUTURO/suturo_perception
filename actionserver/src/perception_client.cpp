//
// Created by Jeremias Thun on 23.12.19.
//

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <suturo_perception_msgs/ExtractObjectInfoAction.h>

int main (int argc, char **argv) {
    ros::init (argc, argv, "test_perception");

    //create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<suturo_perception_msgs::ExtractObjectInfoAction> ac("perception_data", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the acction server to start
    ac.waitForServer(); // will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    //send a goal to the action
    suturo_perception_msgs::ExtractObjectInfoGoal goal;
    goal.visualize = false;
    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    } else {
        ROS_INFO("Action did not finish before the time out");
    }

    //exit
    return 0;
}