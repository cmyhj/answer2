#include <string>

#include "robot_behavior_tree/plugins/condition/if_HP_or_bullet_low.hpp"

namespace nav2_behavior_tree
{

    IfHPOrBulletLowCondition::IfHPOrBulletLowCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf)
    {
        sentry_HP_= config().blackboard->get<double>("sentry_HP");
    }

    BT::NodeStatus IfHPOrBulletLowCondition::tick()
    {
        sentry_HP_= config().blackboard->get<double>("sentry_HP");
        if (sentry_HP_<95)
        {
            std::cout<<"HP小于95"<<std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        std::cout<<"HP大于95"<<std::endl;
        return BT::NodeStatus::FAILURE;
    }


} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IfHPOrBulletLowCondition>("IfHPOrBulletLow");
}
