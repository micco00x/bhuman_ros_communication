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

int main(int argc, char** argv) {
  ros::init(argc, argv, "bhuman_ros_communication");
  ros::NodeHandle nodeHandle("~");

  ROS_INFO("bhuman_ros_communication node started.");

  const in_port_t PORT = 1999;

  int sockfd; // server socket file descriptor
  int cli_sockfd; // socket file descriptor related to connection with client
  struct sockaddr_in serv_addr, cli_addr;

  // Open server socket:
  if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
    perror("Cannot open socket.");
    exit(0);
  }

  // Setup server address structure:
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = PORT;
  serv_addr.sin_addr.s_addr = INADDR_ANY;

  // Bind socket to specified address and port:
  if (bind(sockfd, (struct sockaddr*) &serv_addr, sizeof(serv_addr)) == -1) {
    perror("Cannot bind to specified address and port.");
    exit(0);
  }

  // Listen to incoming connections:
  listen(sockfd, 1);

  // Wait for a connection:
  ROS_INFO("Waiting for a connection...");
  socklen_t clilen = sizeof(cli_addr);
  if ((cli_sockfd = accept(sockfd, (struct sockaddr*) &cli_addr, (socklen_t*) &clilen)) == -1) {
    perror("Cannot accept connection.");
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

  size_t configuration_size = sizeof(double) * 9 + sizeof(argos::Foot);
  size_t buffer_size = configuration_size * footstepPlan.size();
  std::unique_ptr<char[]> footstep_plan_buffer(new char [buffer_size]);

  ROS_INFO("Sending footstep plan...");
  fill_buffer(footstepPlan, &footstep_plan_buffer[0]);

  send_plan_size(cli_sockfd, footstepPlan.size());
  send_buffer(cli_sockfd, &footstep_plan_buffer[0], buffer_size);

  // Close sockets:
  close(cli_sockfd);
  close(sockfd);

  return 0;
}
