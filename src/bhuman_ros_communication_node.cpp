// STL
#include <memory>
#include <thread>

// Socket
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/signal.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>

// ROS
#include <ros/ros.h>

// Argos
#include <argos_msgs/DoubleSupportConfiguration.h>
#include <argos_msgs/FootstepPlan.h>
#include <argos_planning/ArgosRosConverter.hpp>
#include <argos_planning/DoubleSupportConfiguration.hpp>
#include <argos_planning/FootstepPlan.hpp>
#include <argos_planning/enum.hpp>

class TCPWrapper {
 public:
  TCPWrapper(ros::NodeHandle& nodeHandle) {
    nodeHandle.param(
        "footstep_plan_topic",
        footstep_plan_topic_, 
        std::string("/argos_planning/footstep_plan")
    );

    footstepPlanSubscriber_ = nodeHandle.subscribe(
        footstep_plan_topic_, 1,
        &TCPWrapper::footstepPlanCallback, this
    );

    configurationPublisher_ =
        nodeHandle.advertise<argos_msgs::DoubleSupportConfiguration>(
            "double_support_configuration", 1, true
        );
  }

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
    if ((cli_sockfd_ = accept(sockfd_, (struct sockaddr*) &cli_addr,
        (socklen_t*) &clilen)) == -1) {
      return false;
    }

    configuration_thread_ = std::thread(&TCPWrapper::configuration_poll, this);

    return true;
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

  void footstepPlanCallback(
      const argos_msgs::FootstepPlan& footstepPlanMessage) {
    argos::FootstepPlan footstepPlan;
    argos::ArgosRosConverter::fromMessage(footstepPlanMessage, footstepPlan);

    ROS_INFO("Sending footstep plan...");
    if (!send_footstep_plan(footstepPlan)) {
      ROS_ERROR("Cannot send footstep plan.");
    }
  }

  void configuration_poll() {
    struct pollfd pfds[1];
    pfds[0].fd = cli_sockfd_;
    pfds[0].events = POLLIN;

    while (1) {
      // Poll and receive footstep plan if POLLIN flag has been set:
      if (poll(pfds, 1, -1) > 0) {
        if (pfds[0].revents & POLLIN) {
          if (!recvConfiguration()) {
            break;
          }
        }
      } else {
        ROS_ERROR("Client disconnected.");
        break;
      }
    }
  }

  bool recvConfiguration() {
    // Check that client socket file descriptor is valid before receiving the
    // double support configuration:
    if (fcntl(cli_sockfd_, F_GETFD) == -1) {
      ROS_ERROR("Invalid client file descriptor.");
      return false;
    }

    // Read buffer:
    const size_t configuration_size = 9 * sizeof(double) + sizeof(argos::Foot);
    char buffer[configuration_size];
    if (recv(cli_sockfd_, buffer, configuration_size, MSG_WAITALL) == -1) {
      ROS_ERROR("Cannot receive configuration.");
      return false;
    }

    Eigen::Vector4d qL, qR;
    argos::Foot support_foot;
    double h_z;

    double q[8];
    std::memcpy(q, buffer, sizeof(q));
    std::memcpy(&support_foot, buffer + sizeof(q), sizeof(support_foot));
    std::memcpy(&h_z, buffer + sizeof(q) + sizeof(support_foot), sizeof(h_z));
    qL << q[0], q[1], q[2], q[3];
    qR << q[4], q[5], q[6], q[7];

    publishConfiguration(
        argos::DoubleSupportConfiguration(qL, qR, support_foot, h_z)
    );

    return true;
  }

  void publishConfiguration(
      const argos::DoubleSupportConfiguration& configuration) {
    argos_msgs::DoubleSupportConfiguration message;
    argos::ArgosRosConverter::toMessage(configuration, message);
    configurationPublisher_.publish(message);
  }

  int sockfd_; // server socket file descriptor
  int cli_sockfd_; // socket file descriptor related to connection with client

  std::string footstep_plan_topic_;

  ros::Subscriber footstepPlanSubscriber_;

  ros::Publisher configurationPublisher_;

  std::thread configuration_thread_;
}; // end class TCPWrapper



int main(int argc, char** argv) {
  ros::init(argc, argv, "bhuman_ros_communication");
  ros::NodeHandle nodeHandle("~");

  TCPWrapper tcp_wrapper(nodeHandle);

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

  ros::spin();

  return 0;
}
