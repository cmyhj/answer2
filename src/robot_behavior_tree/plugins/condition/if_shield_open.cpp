#include <string>

#include "robot_behavior_tree/plugins/condition/if_shield_open.hpp"

namespace nav2_behavior_tree
{

    IfShieldOpenCondition::IfShieldOpenCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf)
    {
    }

    BT::NodeStatus IfShieldOpenCondition::tick(){
        if(config().blackboard->get<bool>("is_enemy_base_exist")==true || config().blackboard->get<bool>("guess_enemy_base_exist")==true)
        {
            // std::cout<<"基地开花咯"<<std::endl;
            config().blackboard->set<double>("health_threshold",25);
            return BT::NodeStatus::SUCCESS;
        }
        config().blackboard->set<double>("health_threshold",95);
        return BT::NodeStatus::FAILURE;
    }


} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IfShieldOpenCondition>("IfShieldOpen");
}
