#include <string>

#include "robot_behavior_tree/plugins/action/go_base.hpp"


namespace nav2_behavior_tree
{
    //    position_x(0.0),
    //    position_y(0.0)
    GoBaseAction::GoBaseAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        base_pose_.header.stamp = node_->now();
        base_pose_.header.frame_id = "map";
        base_pose_.pose.position.x = 0.0;
        base_pose_.pose.position.y = 0.0;
        base_pose_.pose.position.z = 0.0;
        base_pose_.pose.orientation.x = 0.0;
        base_pose_.pose.orientation.y = 0.0;
        base_pose_.pose.orientation.z = 0.0;
        base_pose_.pose.orientation.w = 1.0;
    }

    BT::NodeStatus GoBaseAction::tick()
    {
        base_pose_.header.stamp = node_->now();
        base_pose_.pose.position.x=config().blackboard->get<double>("base_pose_x");
        base_pose_.pose.position.y=config().blackboard->get<double>("base_pose_y");
        goal_pub_->publish(base_pose_);
        std::cout<<"回基地补给"<<std::endl;
        return BT::NodeStatus::SUCCESS;
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::GoBaseAction>("GoBase");
}
