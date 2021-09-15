// ROS
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "bhuman_ros_communication");
  ros::NodeHandle nodeHandle("~");

  ROS_INFO("bhuman_ros_communication node started.");

  return 0;
}
