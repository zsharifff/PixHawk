#include <gz/sim/System.hh>
#include <gz/transport.hh>

namespace px4::sim
{
  class TopicRemap:
    // This class is a system.
    public gz::sim::System,
    // This class also implements the ISystemPostUpdate interface.
    public gz::sim::ISystemPostUpdate
  {
    public: TopicRemap();

    public: ~TopicRemap() override;

    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;
    private:
      gz::transport::Node _node;
  };
}
