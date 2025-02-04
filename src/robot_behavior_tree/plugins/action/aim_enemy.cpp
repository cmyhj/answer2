#include <string>

#include "robot_behavior_tree/plugins/action/aim_enemy.hpp"


namespace nav2_behavior_tree
{
    //    position_x(0.0),
    //    position_y(0.0)
    AimEnemyAction::AimEnemyAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        enemy_pose.header.stamp = node_->now();
        enemy_pose.header.frame_id = "map";
        enemy_pose.pose.position.x = 0.0;
        enemy_pose.pose.position.y = 0.0;
        enemy_pose.pose.position.z = 0.0;
        enemy_pose.pose.orientation.x = 0.0;
        enemy_pose.pose.orientation.y = 0.0;
        enemy_pose.pose.orientation.z = 0.0;
        enemy_pose.pose.orientation.w = 1.0;
    }

    BT::NodeStatus AimEnemyAction::tick()
    {
        enemy_pose.header.stamp = node_->now();
        enemy_pose.pose.position.x=config().blackboard->get<double>("enemy_pose_x");
        enemy_pose.pose.position.y=config().blackboard->get<double>("enemy_pose_y");
        goal_pub_->publish(enemy_pose);
        std::cout<<"更新目标点"<<enemy_pose.pose.position.x<<" , "<<enemy_pose.pose.position.y<<std::endl;
        return BT::NodeStatus::SUCCESS;
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::AimEnemyAction>("AimEnemy");
}
