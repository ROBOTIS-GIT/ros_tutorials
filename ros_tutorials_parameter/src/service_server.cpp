#include "ros/ros.h"                          // ROS 기본 헤더 파일
#include "ros_tutorials_parameter/SrvTutorial.h"// SrvTutorial 서비스 파일 헤더  (빌드후 자동 생성됨)

#define PLUS            1   // 덧셈
#define MINUS           2   // 빼기
#define MULTIPLICATION  3   // 곱하기
#define DIVISION        4   // 나누기

int g_operator = PLUS;

// 서비스 요청이 있을 경우, 아래의 처리를 수행한다
// 서비스 요청은 req, 서비스 응답은 res로 설정하였다
bool calculation(ros_tutorials_parameter::SrvTutorial::Request &req,
                 ros_tutorials_parameter::SrvTutorial::Response &res)
{
  // 서비스 요청시 받은 a와 b 값을 파라미터값에 따라 연산자를 달리한다.
  // 계산한 후 서비스 응답 값에 저장한다
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

  // 서비스 요청에 사용된 a, b값의 표시 및 서비스 응답에 해당되는 result 값을 출력한다
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.result);

  return true;
}

int main(int argc, char **argv)              // 노드 메인 함수
{
  ros::init(argc, argv, "service_server");   // 노드명 초기화
  ros::NodeHandle nh;                        // 노드 핸들 선언

  nh.setParam("calculation_method", PLUS); // 매개변수 초기설정

  // 서비스 서버 선언,
  // ros_tutorials_parameter 패키지의 SrvTutorial 서비스 파일을 이용한
  // 서비스 서버 ros_tutorials_service_server를 선언한다
  // 서비스명은 ros_tutorial_srv이며 서비스 요청이 있을 때,
  // calculation라는 함수를 실행하라는 설정이다
  ros::ServiceServer ros_tutorials_service_server = nh.advertiseService("ros_tutorial_srv", calculation);

  ROS_INFO("ready srv server!");

  ros::Rate r(10); // 10 hz

  while (ros::ok())
  {
    nh.getParam("calculation_method", g_operator);  // 연산자를 매개변수로부터 받은 값으로 변경한다
    ros::spinOnce();  // 콜백함수 처리루틴
    r.sleep();        // 루틴 반복을 위한 sleep 처리
  }

  return 0;
}
