#include "ros/ros.h"                          // ROS Default Header File
#include "ros_tutorials_topic/MsgTutorial.h"  // MsgTutorial Message File Header. The header file is automatically created when building the package.

// Message callback function. This is a function is called when a topic
// message named 'ros_tutorial_msg' is received. As an input message,
// the 'MsgTutorial' message of the 'ros_tutorials_topic' package is received.
void msgCallback(const ros_tutorials_topic::MsgTutorial::ConstPtr& msg)
{
  ROS_INFO("recieve msg = %d", msg->stamp.sec);   // Prints the 'stamp.sec' message
  ROS_INFO("recieve msg = %d", msg->stamp.nsec);  // Prints the 'stamp.nsec' message
  ROS_INFO("recieve msg = %d", msg->data);        // Prints the 'data' message
}

int main(int argc, char **argv)                         // Node Main Function
{
  ros::init(argc, argv, "topic_subscriber");            // Initializes Node Name

  ros::NodeHandle nh;                                   // Node handle declaration for communication with ROS system

  // Declares subscriber. Create subscriber 'ros_tutorial_sub' using the 'MsgTutorial'
  // message file from the 'ros_tutorials_topic' package. The topic name is
  // 'ros_tutorial_msg' and the size of the publisher queue is set to 100.
  ros::Subscriber ros_tutorial_sub = nh.subscribe("ros_tutorial_msg", 100, msgCallback);

  // A function for calling a callback function, waiting for a message to be
  // received, and executing a callback function when it is received.
  ros::spin();

  return 0;
}
