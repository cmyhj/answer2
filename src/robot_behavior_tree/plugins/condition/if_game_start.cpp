#include <string>

#include "robot_behavior_tree/plugins/condition/if_game_start.hpp"

namespace nav2_behavior_tree
{

    IfGameStartCondition::IfGameStartCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf)
    {
    }

    BT::NodeStatus IfGameStartCondition::tick(){
        if(config().blackboard->get<double>("sentry_HP")==0)
        {
            std::cout<<"等待游戏开始"<<std::endl;
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }


} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IfGameStartCondition>("IfGameStart");
}
