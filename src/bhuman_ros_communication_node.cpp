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

class TCPWrapper {
 public:
  ~TCPWrapper() {
    // Close sockets:
    close(cli_sockfd_);
    close(sockfd_);
  }

  bool open() {
    return (sockfd_ = socket(AF_INET, SOCK_STREAM, 0)) != -1;
  }

  bool bind_to_port(in_port_t port) {
    struct sockaddr_in serv_addr;

    // Setup server address structure:
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = port;
    serv_addr.sin_addr.s_addr = INADDR_ANY;

    return (bind(sockfd_, (struct sockaddr*) &serv_addr, sizeof(serv_addr)) != -1);
  }

  bool wait_for_connection() {
    struct sockaddr_in cli_addr;

    listen(sockfd_, 1);

    socklen_t clilen = sizeof(cli_addr);
    return (cli_sockfd_ = accept(sockfd_, (struct sockaddr*) &cli_addr, (socklen_t*) &clilen)) != -1;
  }

  bool send_footstep_plan(const argos::FootstepPlan& footstepPlan) {
    size_t configuration_size = sizeof(double) * 9 + sizeof(argos::Foot);
    size_t buffer_size = configuration_size * footstepPlan.size();
    std::unique_ptr<char[]> footstep_plan_buffer(new char [buffer_size]);

    fill_buffer(footstepPlan, &footstep_plan_buffer[0]);

    ssize_t ss = send_plan_size(cli_sockfd_, footstepPlan.size());
    if (ss == -1) {
      return false;
    }
    ss = send_buffer(cli_sockfd_, &footstep_plan_buffer[0], buffer_size);
    return ss != -1;
  }

 private:
  void fill_buffer(const argos::FootstepPlan& footstepPlan, char* buffer) {
    char* buffer_ptr = buffer;
    for (const auto& configuration : footstepPlan.getPlan()) {
      const auto& qL = configuration.getqL();
      const auto& qR = configuration.getqR();
      argos::Foot support_foot = configuration.getSupportFoot();
      double h_z = configuration.getSwingFootTrajectoryHeight();
      double q[8] = {qL.x(), qL.y(), qL.z(), qL.w(),
                    qR.x(), qR.y(), qR.z(), qR.w()};
      // Copy q to buffer:
      std::memcpy(buffer_ptr, q, sizeof(q));
      buffer_ptr += sizeof(q);
      // Copy support foot to buffer:
      std::memcpy(buffer_ptr, &support_foot, sizeof(support_foot));
      buffer_ptr += sizeof(support_foot);
      // Copy h_z to buffer:
      std::memcpy(buffer_ptr, &h_z, sizeof(h_z));
      buffer_ptr += sizeof(h_z);
    }
  }

  ssize_t send_plan_size(
      int client_socket_descriptor,
      uint32_t plan_size) {
    return send(
        client_socket_descriptor,
        &plan_size,
        sizeof(plan_size),
        0
    );
  }

  ssize_t send_buffer(
    int client_socket_descriptor,
    char* buffer,
    size_t buffer_size) {
    return send(
        client_socket_descriptor,
        buffer,
        buffer_size,
        0
    );
  }

  int sockfd_; // server socket file descriptor
  int cli_sockfd_; // socket file descriptor related to connection with client
}; // end class TCPWrapper



int main(int argc, char** argv) {
  ros::init(argc, argv, "bhuman_ros_communication");
  ros::NodeHandle nodeHandle("~");

  TCPWrapper tcp_wrapper;

  ROS_INFO("bhuman_ros_communication node started.");

  const in_port_t PORT = 1999;

  // Open server socket:
  if (!tcp_wrapper.open()) {
    ROS_ERROR("Cannot open socket.");
    exit(0);
  }

  // Bind socket to specified port:
  if (!tcp_wrapper.bind_to_port(PORT)) {
    ROS_ERROR("Cannot bind to specified port.");
    exit(0);
  }

  ROS_INFO("Waiting for a connection...");
  if (!tcp_wrapper.wait_for_connection()) {
    ROS_ERROR("Cannot establish a connection.");
  }

  ROS_INFO("Connection established.");

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

  ROS_INFO("Sending footstep plan...");
  if (!tcp_wrapper.send_footstep_plan(footstepPlan)) {
    ROS_ERROR("Cannot send footstep plan.");
  }

  return 0;
}
