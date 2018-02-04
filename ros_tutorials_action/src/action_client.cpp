#include <ros/ros.h>                              // ROS Default Header File
#include <actionlib/client/simple_action_client.h>// action Library Header File
#include <actionlib/client/terminal_state.h>      // Action Goal Status Header File
#include <ros_tutorials_action/FibonacciAction.h> // FibonacciAction Action File Header

int main (int argc, char **argv)          // Node Main Function
{
  ros::init(argc, argv, "action_client"); // Node Name Initialization

  // Action Client Declaration (Action Name: ros_tutorial_action)
  actionlib::SimpleActionClient<ros_tutorials_action::FibonacciAction> ac("ros_tutorial_action", true);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer(); //wait for the action server to start, will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  ros_tutorials_action::FibonacciGoal goal; // Declare Action Goal
  goal.order = 20;    // Set Action Goal (Process the Fibonacci sequence 20 times)
  ac.sendGoal(goal);  // Transmit Action Goal

  // Set action time limit (set to 30 seconds)
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  // Process when action results are received within the time limit for achieving the action goal
  if (finished_before_timeout)
  {
    // Receive action target status value and display on screen
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
