#include "ros/ros.h"                                    // ROS 기본 헤더파일
#include "ros_tutorials_topic/MsgTutorial.h"                  // msgTutorial 메시지 파일 헤더 (빌드후 자동 생성됨)

int main(int argc, char **argv)                         // 노드 메인 함수
{
  ros::init(argc, argv, "topic_publisher");             // 노드명 초기화
  ros::NodeHandle nh;                                   // ROS 시스템과 통신을 위한 노드 핸들 선언

  // 퍼블리셔 선언, ros_tutorials_topic 패키지의 MsgTutorial 메시지 파일을 이용한
  // 퍼블리셔 ros_tutorial_pub 를 작성한다. 토픽명은 "ros_tutorial_msg" 이며,
  // 퍼블리셔 큐(queue) 사이즈를 100개로 설정한다는 것이다
  ros::Publisher ros_tutorial_pub = nh.advertise<ros_tutorials_topic::MsgTutorial>("ros_tutorial_msg", 100);

  // 루프 주기를 설정한다. "10" 이라는 것은 10Hz를 말하는 것으로 0.1초 간격으로 반복된다
  ros::Rate loop_rate(10);

  ros_tutorials_topic::MsgTutorial msg;     // MsgTutorial 메시지 파일 형식으로 msg 라는 메시지를 선언
  int count = 0;                            // 메시지에 사용될 변수 선언

  while (ros::ok())
  {
    msg.stamp = ros::Time::now();           // 현재 시간을 msg의 하위 stamp 메시지에 담는다
    msg.data  = count;                      // count라는 변수 값을 msg의 하위 data 메시지에 담는다

    ROS_INFO("send msg = %d", msg.stamp.sec);   // ROS_INFO 라는 ROS 함수를 이용하여 count 변수를 표시한다
    ROS_INFO("send msg = %d", msg.stamp.nsec);  // ROS_INFO 라는 ROS 함수를 이용하여 count 변수를 표시한다
    ROS_INFO("send msg = %d", msg.data);        // ROS_INFO 라는 ROS 함수를 이용하여 count 변수를 표시한다

    ros_tutorial_pub.publish(msg);          // 메시지를 발행한다. 약 0.1초 간격으로 발행된다

    loop_rate.sleep();                      // 위에서 정한 루프 주기에 따라 슬립에 들어간다

    ++count;                                // count 변수 1씩 증가
  }

  return 0;
}
