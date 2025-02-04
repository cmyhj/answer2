#include <string>

#include "robot_behavior_tree/plugins/action/update_map_info.hpp"
namespace nav2_behavior_tree
{
    UpdateMapInfoAction::UpdateMapInfoAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        enemy_pose_sub_= node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/map_info",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&UpdateMapInfoAction::mapInfoCallback, this, std::placeholders::_1),
            sub_option);
    }

    BT::NodeStatus UpdateMapInfoAction::tick()
    {
        callback_group_executor_.spin_some();
// pose.header.stamp = node_->now();
//         pose.header.frame_id = "map";
//         pose.pose.position.x = 2.0;
//         pose.pose.position.y = 2.0;
//         pose.pose.position.z = 0.0;
//         pose.pose.orientation.x = 0.0;
//         pose.pose.orientation.y = 0.0;
//         pose.pose.orientation.z = 0.0;
//         pose.pose.orientation.w = 1.0;
        // getInput<geometry_msgs::msg::PoseStamped>("enemy_pose",enemy_pose);
        // goal_pub_->publish(pose);
        // std::cout<<"更新目标点"<<goal_pose.pose.position.x<<" , "<<goal_pose.pose.position.y<<std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    void UpdateMapInfoAction::mapInfoCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // 打印接收到的 PoseStamped 消息的相关信息
        std::cout << "Received enemy_pose update:" << std::endl;
        std::cout << "Position (x, y, z): (" 
                << msg->pose.position.x << ", " 
                << msg->pose.position.y << ", " 
                << msg->pose.position.z << ")" << std::endl;

        // 将消息设置为输出
        config().blackboard->set<double>("enemy_pose_x", msg->pose.position.x);
        config().blackboard->set<double>("enemy_pose_y", msg->pose.position.y);
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::UpdateMapInfoAction>("UpdateMapInfo");
}
