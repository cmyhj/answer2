#include <string>

#include "robot_behavior_tree/plugins/condition/if_is_transfering.hpp"

namespace nav2_behavior_tree
{

    IfIsTransferingCondition::IfIsTransferingCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf)
    {
        is_transfering_= config().blackboard->get<bool>("is_transfering");
    }

    BT::NodeStatus IfIsTransferingCondition::tick(){
        is_transfering_= config().blackboard->get<bool>("is_transfering");
        if (is_transfering_==true)
        {
            std::cout<<"传送中..."<<std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
        
    }


} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IfIsTransferingCondition>("IfIsTransfering");
}
