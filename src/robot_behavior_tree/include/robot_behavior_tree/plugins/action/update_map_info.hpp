#ifndef UPDATE_MAP_INFO_HPP_
#define UPDATE_MAP_INFO_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "robot_msgs/msg/map_info.hpp"
#include "robot_msgs/msg/map_info_msgs.hpp"
enum TargetType : uint8_t {
    STAR,
    BASE,
    ENEMY_BASE,
    PURPLEENTRY,
    GREENENTRY,
    SENTRY,
    ENEMY
};

namespace nav2_behavior_tree
{
    class UpdateMapInfoAction : public BT::SyncActionNode
    {
    public:
        UpdateMapInfoAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        UpdateMapInfoAction() = delete;

        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts()
        {
            return {
                BT::OutputPort<double>("star_pose_x", "Star position x"),
                BT::OutputPort<double>("star_pose_y", "Star position y"),
                BT::OutputPort<double>("base_pose_x", "Base position x"),
                BT::OutputPort<double>("base_pose_y", "Base position y"),
                BT::OutputPort<double>("enemy_base_pose_x", "Enemy base position x"),
                BT::OutputPort<double>("enemy_base_pose_y", "Enemy base position y"),
                BT::OutputPort<double>("purple_entry_pose_x", "Purple entry position x"),
                BT::OutputPort<double>("purple_entry_pose_y", "Purple entry position y"),
                BT::OutputPort<double>("green_entry_pose_x", "Green entry position x"),
                BT::OutputPort<double>("green_entry_pose_y", "Green entry position y"),
                BT::OutputPort<double>("sentry_pose_x", "Sentry position x"),
                BT::OutputPort<double>("sentry_pose_y", "Sentry position y"),
                BT::OutputPort<double>("enemy_pose_x", "Enemy position x"),
                BT::OutputPort<double>("enemy_pose_y", "Enemy position y"),
                BT::OutputPort<int>("enemy_num", "Number of enemies"),
                BT::OutputPort<double>("sentry_HP", "Sentry health points"),
                BT::OutputPort<bool>("is_transfering", "is sentry transfering?"),
                BT::OutputPort<bool>("is_star_exist", "is star exist?"),
                BT::OutputPort<bool>("is_base_exist", "is base exist?"),
                BT::OutputPort<bool>("is_enemy_base_exist", "is enemy base exist?"),
                BT::OutputPort<bool>("is_purple_entry_out_of_range", "is purple entry out of range?"),
                BT::OutputPort<bool>("is_green_entry_out_of_range", "is green entry out of range?"),
                BT::OutputPort<bool>("is_sentry_out_of_range", "is sentry out of range?"),
                BT::OutputPort<bool>("is_enemy_out_of_range", "is enemy out of range?")
            };
        }

    private:
        void mapInfoCallback(const robot_msgs::msg::MapInfoMsgs::SharedPtr msg);
        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        geometry_msgs::msg::PoseStamped enemy_pose;
        rclcpp::Subscription<robot_msgs::msg::MapInfoMsgs>::SharedPtr map_info_sub_;
    };

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
