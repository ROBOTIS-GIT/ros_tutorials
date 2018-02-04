#include "ros/ros.h"                            // ROS Default Header File
#include "ros_tutorials_parameter/SrvTutorial.h"// action Library Header File

#define PLUS            1   // Addition
#define MINUS           2   // Subtraction
#define MULTIPLICATION  3   // Multiplication
#define DIVISION        4   // Division

int g_operator = PLUS;

// The process below is performed if there is a service request
// The service request is declared as 'req', and the service response is declared as 'res'
bool calculation(ros_tutorials_parameter::SrvTutorial::Request &req,
                 ros_tutorials_parameter::SrvTutorial::Response &res)
{
  // The operator will be selected according to the parameter value and calculate 'a' and 'b',
  // which were received upon the service request.
  // The result is stored as the Response value.
  switch(g_operator)
  {
    case PLUS:
         res.result = req.a + req.b; break;
    case MINUS:
         res.result = req.a - req.b; break;
    case MULTIPLICATION:
         res.result = req.a * req.b; break;
    case DIVISION:
         if(req.b == 0)
         {
           res.result = 0; break;
         }
         else
         {
           res.result = req.a / req.b; break;
         }
    default:
         res.result = req.a + req.b; break;
  }

  // Displays the values of 'a' and 'b' used in the service request, and the 'result' value
  // corresponding to the service response.
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.result);

  return true;
}

int main(int argc, char **argv)             // Node Main Function
{
  ros::init(argc, argv, "service_server");  // Initializes Node Name
  ros::NodeHandle nh;                       // Node handle declaration

  nh.setParam("calculation_method", PLUS);  // Reset Parameter Settings

  // Declare service server 'service_server' using the 'SrvTutorial' service file
  // in the 'ros_tutorials_service' package. The service name is 'ros_tutorial_srv' and
  // it is set to execute a 'calculation' function when a service is requested.
  ros::ServiceServer ros_tutorials_service_server = nh.advertiseService("ros_tutorial_srv", calculation);

  ROS_INFO("ready srv server!");

  ros::Rate r(10);  // 10 hz

  while (ros::ok())
  {
    nh.getParam("calculation_method", g_operator);  // Select the operator according to the value received from the parameter.
    ros::spinOnce();  // Callback function process routine
    r.sleep();        // Sleep for routine iteration
  }

  return 0;
}
