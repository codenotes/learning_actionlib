cmake_minimum_required(VERSION 2.8.3)
project(learning_actionlib)

find_package(catkin REQUIRED COMPONENTS roscpp genmsg actionlib actionlib_msgs)
find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES Fibonacci.action
)

add_message_files(
  DIRECTORY msg
  FILES
  beaconSetting.msg
  beaconSettings.msg
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
  message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(
    fibonacci_server src/fibonacci_server.cpp
)

add_executable(
    fibonacci_client src/fibonacci_client.cpp
)

target_link_libraries(
  fibonacci_server
  ${catkin_LIBRARIES}
)

target_link_libraries(
  fibonacci_client
  ${catkin_LIBRARIES}
)

add_dependencies(
  fibonacci_server
  ${learning_actionlib_EXPORTED_TARGETS}
)

add_dependencies(
  fibonacci_client
  ${learning_actionlib_EXPORTED_TARGETS}
)

add_definitions(-std=c++11)
