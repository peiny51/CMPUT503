cmake_minimum_required(VERSION 2.8.3)
project(exercise_two)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_package()