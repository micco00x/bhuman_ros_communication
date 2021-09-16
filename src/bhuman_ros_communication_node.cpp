// STL
#include <memory>

// Socket
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/signal.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <fcntl.h>
#include <unistd.h>

// ROS
#include <ros/ros.h>

// Argos
#include <argos_planning/DoubleSupportConfiguration.hpp>
#include <argos_planning/FootstepPlan.hpp>
#include <argos_planning/enum.hpp>

void fill_buffer(const argos::FootstepPlan& footstepPlan, char* buffer) {
  char* buffer_ptr = buffer;
  for (const auto& configuration : footstepPlan.getPlan()) {
    const auto& qL = configuration.getqL();
    const auto& qR = configuration.getqR();
    argos::Foot support_foot = configuration.getSupportFoot();
    double h_z = configuration.getSwingFootTrajectoryHeight();
    // Copy qL to buffer:
    std::memcpy(buffer_ptr, &qL.x(), sizeof(qL.x()));
    buffer_ptr += sizeof(qL.x());
    std::memcpy(buffer_ptr, &qL.y(), sizeof(qL.y()));
    buffer_ptr += sizeof(qL.y());
    std::memcpy(buffer_ptr, &qL.z(), sizeof(qL.z()));
    buffer_ptr += sizeof(qL.z());
    std::memcpy(buffer_ptr, &qL.w(), sizeof(qL.w()));
    buffer_ptr += sizeof(qL.w());
    // Copy qR to buffer:
    std::memcpy(buffer_ptr, &qR.x(), sizeof(qR.x()));
    buffer_ptr += sizeof(qR.x());
    std::memcpy(buffer_ptr, &qR.y(), sizeof(qR.y()));
    buffer_ptr += sizeof(qR.y());
    std::memcpy(buffer_ptr, &qR.z(), sizeof(qR.z()));
    buffer_ptr += sizeof(qR.z());
    std::memcpy(buffer_ptr, &qR.w(), sizeof(qR.w()));
    buffer_ptr += sizeof(qR.w());
    // Copy support foot to buffer:
    std::memcpy(buffer_ptr, &support_foot, sizeof(support_foot));
    buffer_ptr += sizeof(support_foot);
    // Copy h_z to buffer:
    std::memcpy(buffer_ptr, &h_z, sizeof(h_z));
    buffer_ptr += sizeof(h_z);
  }
}

ssize_t send_buffer_size(
    int server_socket_descriptor,
    uint32_t buffer_size) {
  return send(
      server_socket_descriptor,
      &buffer_size,
      sizeof(buffer_size),
      0
  );
}

ssize_t send_buffer(
  int server_socket_descriptor,
  char* buffer) {
  return send(
      server_socket_descriptor,
      buffer,
      sizeof(buffer),
      0
  );
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "bhuman_ros_communication");
  ros::NodeHandle nodeHandle("~");

  ROS_INFO("bhuman_ros_communication node started.");

  argos::FootstepPlan footstepPlan;
  footstepPlan.setFrameId("odom");
  footstepPlan.setTimestamp(ros::Time::now().toSec());
  footstepPlan.push_back(argos::DoubleSupportConfiguration(
      Eigen::Vector4d(0.0,  0.05, 0.0, 0.0),
      Eigen::Vector4d(0.0, -0.05, 0.0, 0.0),
      argos::Foot::RIGHT,
      0.0
  ));
  footstepPlan.push_back(argos::DoubleSupportConfiguration(
      Eigen::Vector4d(0.1,  0.05, 0.0, 0.0),
      Eigen::Vector4d(0.0, -0.05, 0.0, 0.0),
      argos::Foot::LEFT,
      0.025
  ));
  footstepPlan.push_back(argos::DoubleSupportConfiguration(
      Eigen::Vector4d(0.1,  0.05, 0.0, 0.0),
      Eigen::Vector4d(0.2, -0.05, 0.0, 0.0),
      argos::Foot::RIGHT,
      0.025
  ));
  footstepPlan.push_back(argos::DoubleSupportConfiguration(
      Eigen::Vector4d(0.3,  0.05, 0.0, 0.0),
      Eigen::Vector4d(0.2, -0.05, 0.0, 0.0),
      argos::Foot::LEFT,
      0.025
  ));
  footstepPlan.push_back(argos::DoubleSupportConfiguration(
      Eigen::Vector4d(0.3,  0.05, 0.0, 0.0),
      Eigen::Vector4d(0.3, -0.05, 0.0, 0.0),
      argos::Foot::RIGHT,
      0.025
  ));

  size_t configuration_size = sizeof(double) * 9 + sizeof(argos::Foot);
  std::unique_ptr<char[]> footstep_plan_buffer(
      new char [configuration_size * footstepPlan.size()]
  );

  fill_buffer(footstepPlan, footstep_plan_buffer.get());

  return 0;
}
