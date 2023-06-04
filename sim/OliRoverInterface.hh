#ifndef SYSTEM_PLUGIN_OliRoverInterface_HH_
#define SYSTEM_PLUGIN_OliRoverInterface_HH_

// The only required include in the header is this one.
// All others will depend on what your plugin does.
#include <gz/sim/System.hh>

// It's good practice to use a custom namespace for your project.
namespace oli_rover_interface
{
  // This is the main plugin's class. It must inherit from System and at least
  // one other interface.
  // Here we use `ISystemPostUpdate`, which is used to get results after
  // physics runs. The opposite of that, `ISystemPreUpdate`, would be used by
  // plugins that want to send commands.
  class OliRoverInterface : public gz::sim::System,
                            public gz::sim::ISystemConfigure,
                            public gz::sim::ISystemPreUpdate,
                            public gz::sim::ISystemPostUpdate
  {
    // Plugins inheriting ISystemPostUpdate must implement the PostUpdate
    // callback. This is called at every simulation iteration after the physics
    // updates the world. The _info variable provides information such as time,
    // while the _ecm provides an interface to all entities and components in
    // simulation.
  public:
    void PostUpdate(const gz::sim::UpdateInfo &_info,
                    const gz::sim::EntityComponentManager &_ecm) override;


  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

  public:
    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &) override;

  private:
    gz::sim::Entity targetEntity;

    double leftWheelSpeed = {0.0};
    double rightWheelSpeed = {0.0};

    gz::sim::Entity leftJoint;
    gz::sim::Entity rightJoint;

    gz::transport::Node node;

    void imu_callback(const gz::msgs::IMU &_msg);

    int sockfd;
    const int PORT = 1337;
    static const int buffer_size = 1024;
    char buffer[buffer_size];

    void setup_udp_server();
    void read_udp_data();
  };
}
#endif
