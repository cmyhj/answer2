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
        map_info_sub_= node_->create_subscription<robot_msgs::msg::MapInfoMsgs>(
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

    void UpdateMapInfoAction::mapInfoCallback(const robot_msgs::msg::MapInfoMsgs::SharedPtr msg)
    {
        // 将消息设置为输出
        config().blackboard->set<double>("star_pose_x", msg->map_info[STAR].pos.x);
        config().blackboard->set<double>("star_pose_y", msg->map_info[STAR].pos.y);
        config().blackboard->set<double>("base_pose_x", msg->map_info[BASE].pos.x);
        config().blackboard->set<double>("base_pose_y", msg->map_info[BASE].pos.y);
        config().blackboard->set<double>("enemy_base_pose_x", msg->map_info[ENEMY_BASE].pos.x);
        config().blackboard->set<double>("enemy_base_pose_y", msg->map_info[ENEMY_BASE].pos.y);
        config().blackboard->set<double>("purple_entry_pose_x", msg->map_info[PURPLEENTRY].pos.x);
        config().blackboard->set<double>("purple_entry_pose_y", msg->map_info[PURPLEENTRY].pos.y);
        config().blackboard->set<double>("green_entry_pose_x", msg->map_info[GREENENTRY].pos.x);
        config().blackboard->set<double>("green_entry_pose_y", msg->map_info[GREENENTRY].pos.y);
        config().blackboard->set<double>("sentry_pose_x", msg->map_info[SENTRY].pos.x);
        config().blackboard->set<double>("sentry_pose_y", msg->map_info[SENTRY].pos.y);
        config().blackboard->set<double>("enemy_pose_x", msg->map_info[ENEMY].pos.x);
        config().blackboard->set<double>("enemy_pose_y", msg->map_info[ENEMY].pos.y);
        config().blackboard->set<int>("enemy_num", msg->enemy_num);
        config().blackboard->set<double>("sentry_HP", msg->sentry_hp);
        config().blackboard->set<bool>("is_transfering", msg->is_transfering);
        config().blackboard->set<bool>("is_star_exist", msg->map_info[STAR].is_exist_and_out_range);
        config().blackboard->set<bool>("is_base_exist", msg->map_info[BASE].is_exist_and_out_range);
        config().blackboard->set<bool>("is_enemy_base_exist", msg->map_info[ENEMY_BASE].is_exist_and_out_range);
        config().blackboard->set<bool>("is_purple_entry_out_of_range", msg->map_info[PURPLEENTRY].is_exist_and_out_range);
        config().blackboard->set<bool>("is_green_entry_out_of_range", msg->map_info[GREENENTRY].is_exist_and_out_range);
        config().blackboard->set<bool>("is_sentry_out_of_range", msg->map_info[SENTRY].is_exist_and_out_range);
        config().blackboard->set<bool>("is_enemy_out_of_range", msg->map_info[ENEMY].is_exist_and_out_range);
        config().blackboard->set<bool>("is_bullet_low", msg->is_bullet_low);
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::UpdateMapInfoAction>("UpdateMapInfo");
}
