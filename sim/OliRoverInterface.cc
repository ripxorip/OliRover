// We'll use a string and the gzmsg command below for a brief example.
// Remove these includes if your plugin doesn't need them.
#include <string>
#include <gz/common/Console.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointPositionLimitsCmd.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/ParentEntity.hh>

#include <gz/transport/Node.hh>
#include <gz/msgs/imu_sensor.pb.h>

#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

// This header is required to register plugins. It's good practice to place it
// in the cc file, like it's done here.
#include <gz/plugin/Register.hh>

// Don't forget to include the plugin's header.
#include "OliRoverInterface.hh"

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
GZ_ADD_PLUGIN(
    oli_rover_interface::OliRoverInterface,
    gz::sim::System,
    oli_rover_interface::OliRoverInterface::ISystemConfigure,
    oli_rover_interface::OliRoverInterface::ISystemPreUpdate,
    oli_rover_interface::OliRoverInterface::ISystemPostUpdate)

using namespace oli_rover_interface;
using namespace gz;
using namespace sim;

void OliRoverInterface::PostUpdate(const gz::sim::UpdateInfo &_info,
                                   const gz::sim::EntityComponentManager &_ecm)
{
  this->leftWheelSpeed = 4.00;
  this->rightWheelSpeed = -4.00;
}

void OliRoverInterface::PreUpdate(const gz::sim::UpdateInfo &_info,
                                  gz::sim::EntityComponentManager &_ecm)
{
  {
    auto vel = _ecm.Component<components::JointVelocityCmd>(this->leftJoint);
    if (vel == nullptr)
    {
      _ecm.CreateComponent(this->leftJoint,
                           components::JointVelocityCmd({this->leftWheelSpeed}));
    }
    else
    {
      *vel = components::JointVelocityCmd({this->leftWheelSpeed});
    }
  }
  {
    auto vel = _ecm.Component<components::JointVelocityCmd>(this->rightJoint);
    if (vel == nullptr)
    {
      _ecm.CreateComponent(this->rightJoint,
                           components::JointVelocityCmd({this->rightWheelSpeed}));
    }
    else
    {
      *vel = components::JointVelocityCmd({this->rightWheelSpeed});
    }
  }
}

void OliRoverInterface::Configure(const gz::sim::Entity &_entity,
                                  const std::shared_ptr<const sdf::Element> &,
                                  gz::sim::EntityComponentManager &_ecm,
                                  gz::sim::EventManager &)
{
  this->targetEntity = _entity;
  auto joints = _ecm.EntitiesByComponents(
      components::ParentEntity(this->targetEntity), components::Joint());

  for (auto joint : joints)
  {
    auto jointType = _ecm.Component<components::JointType>(joint);
    if (!gz::sim::removeParentScope(::sim::scopedName(joint, _ecm, "::", false), "::").compare("left_wheel_joint"))
    {
      this->leftJoint = joint;
    }
    else if (!gz::sim::removeParentScope(::sim::scopedName(joint, _ecm, "::", false), "::").compare("right_wheel_joint"))
    {
      this->rightJoint = joint;
    }
  }

  std::string topic_sub = "/imu";
  this->node.Subscribe(topic_sub, &OliRoverInterface::imu_callback, this);

  this->setup_udp_server();
}

void OliRoverInterface::imu_callback(const gz::msgs::IMU &_msg)
{
  /* This will be the syncronization */
  gzmsg << "Received IMU message: " << _msg.linear_acceleration().x() << std::endl;
  /* To get actuator control */
  read_udp_data();
  /* To send sensor data to the simulator */
  // send_udp_data();
}

void OliRoverInterface::read_udp_data()
{
  struct sockaddr_in server_addr, client_addr;
  socklen_t addr_len = sizeof(client_addr);

  // Check if data is available
  int bytes_available;
  ioctl(sockfd, FIONREAD, &bytes_available);

  if (bytes_available > 0)
  {
    // Receive message
    ssize_t recv_len = recvfrom(sockfd, (char *)buffer, buffer_size, 0, (struct sockaddr *)&client_addr, &addr_len);
    buffer[recv_len] = '\0';
    std::cout << "Client: " << buffer << std::endl;
  }
}

void OliRoverInterface::setup_udp_server()
{
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
  {
    perror("socket creation failed");
    exit(EXIT_FAILURE);
  }

  struct sockaddr_in server_addr, client_addr;

  // Configure server address
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;
  server_addr.sin_port = htons(PORT);

  // Bind socket to the specified address and port
  if (bind(sockfd, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
  {
    perror("bind failed");
    exit(EXIT_FAILURE);
  }

  // Set socket to non-blocking mode
  int flags = fcntl(sockfd, F_GETFL, 0);
  fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
}