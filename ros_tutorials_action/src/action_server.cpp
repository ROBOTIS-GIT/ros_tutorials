#include <ros/ros.h>                              // ROS Default Header File
#include <actionlib/server/simple_action_server.h>// action Library Header File
#include <ros_tutorials_action/FibonacciAction.h> // FibonacciAction Action File Header

class FibonacciAction
{
protected:

  ros::NodeHandle nh_;  // Node handle declaration
  actionlib::SimpleActionServer<ros_tutorials_action::FibonacciAction> as_; // Action server declaration, NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_; // Use as action name
  // Create messages that are used to published feedback/result
  ros_tutorials_action::FibonacciFeedback feedback_;
  ros_tutorials_action::FibonacciResult result_;

public:
  // Initialize action server (Node handle, action name, action callback function)
  FibonacciAction(std::string name) :
    as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~FibonacciAction(void)
  {
  }

  // A function that receives an action goal message and performs a specified
  // action (in this example, a Fibonacci calculation).
  void executeCB(const ros_tutorials_action::FibonacciGoalConstPtr &goal)
  {
    ros::Rate r(1);       // Loop Rate: 1Hz
    bool success = true;  // Used as a variable to store the success or failure of an action

    // Setting Fibonacci sequence initialization,
    // add first (0) and second message (1) of feedback.
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    // Notify the user of action name, goal, initial two values of Fibonacci sequence
    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    // Action contents
    for(int i=1; i<=goal->order; i++)
    {
      // Confirm action cancellation from action client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        // Notify action cancellation
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // Action cancellation and consider action as failure and save to variable
        as_.setPreempted();
        success = false;
        break;
      }
      // Store the sum of current Fibonacci number and the previous number in the feedback
      // while there is no action cancellation or the action target value is reached.
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    // If the action target value is reached,
    // transmit current Fibonacci sequence as the result value.
    if(success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};

int main(int argc, char** argv)                     // Node Main Function
{
  ros::init(argc, argv, "action_server");           // Initializes Node Name
  FibonacciAction fibonacci("ros_tutorial_action"); // Fibonacci Declaration (Action Name: ros_tutorial_action)
  ros::spin();                                      // Wait to receive action goal
  return 0;
}
