cmake_minimum_required(VERSION 2.8.3)
project(ros_tutorials_parameter)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs roscpp)

add_service_files(FILES SrvTutorial.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  LIBRARIES ros_tutorials_parameter
  CATKIN_DEPENDS std_msgs roscpp
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(service_server_with_parameter src/service_server.cpp)
add_dependencies(service_server_with_parameter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(service_server_with_parameter ${catkin_LIBRARIES})

add_executable(service_client_with_parameter src/service_client.cpp)
add_dependencies(service_client_with_parameter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(service_client_with_parameter ${catkin_LIBRARIES})
