#include "topicremap_plugin.hh"

#include <gz/plugin/RegisterMore.hh>
#include <gz/transport/NodeOptions.hh>


GZ_ADD_PLUGIN(
    px4::sim::TopicRemap,
    gz::sim::System
)

using namespace px4::sim;

TopicRemap::TopicRemap()
{
  std::string target_topic = "/bar";
  _node.Options().TopicRemap("/model/x500_0/servo_0", target_topic);
}

TopicRemap::~TopicRemap()
{
}

void TopicRemap::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  gzmsg << "TopicRemap::PostUpdate" << std::endl;
}
