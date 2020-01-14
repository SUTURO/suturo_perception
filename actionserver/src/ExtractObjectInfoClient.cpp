#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <suturo_perception_msgs/ExtractObjectInfoAction.h>
#include <PerceptionServer.h>

int main (int argc, char **argv) {
    ros::init (argc, argv, "test_perception");

    actionlib::SimpleActionClient<suturo_perception_msgs::ExtractObjectInfoAction> ac(EOI_NAME, true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

    ROS_INFO("Action server started, sending goal.");
    suturo_perception_msgs::ExtractObjectInfoGoal goal;
    goal.visualize = true;
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