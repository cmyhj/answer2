#include <string>

#include "robot_behavior_tree/plugins/condition/is_have_key.hpp"

namespace nav2_behavior_tree
{

    IsHaveKeyCondition::IsHaveKeyCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf)
    {
        enemy_num= config().blackboard->get<int>("enemy_num");
    }

    BT::NodeStatus IsHaveKeyCondition::tick(){
        enemy_num= config().blackboard->get<int>("enemy_num");
        if (enemy_num>0)
        {
            std::cout<<"敌人数量大于0"<<std::endl;
            return BT::NodeStatus::FAILURE;
        }
        std::cout<<"敌人数量等于0"<<std::endl;
        return BT::NodeStatus::SUCCESS;
    }


} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IsHaveKeyCondition>("IsHaveKey");
}
