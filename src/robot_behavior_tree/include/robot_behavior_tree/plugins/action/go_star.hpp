#ifndef GO_STAR_HPP_
#define GO_STAR_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <example_interfaces/msg/int64.hpp>


namespace nav2_behavior_tree
{
    class GoStarAction : public BT::SyncActionNode
    {
    public:
        GoStarAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        GoStarAction() = delete;

        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts()
        {
            return {
                    BT::InputPort<double>("star_pose_x", "Star position x"),
                    BT::InputPort<double>("star_pose_y", "Star position y"),
                    BT::InputPort<double>("purple_entry_pose_x", "Purple entry position x"),
                    BT::InputPort<double>("purple_entry_pose_y", "Purple entry position y"),
                    BT::InputPort<double>("green_entry_pose_x", "Green entry position x"),
                    BT::InputPort<double>("green_entry_pose_y", "Green entry position y"),
                    BT::InputPort<bool>("is_purple_entry_out_of_range", "is purple entry out of range?"),
                    BT::InputPort<bool>("is_green_entry_out_of_range", "is green entry out of range?"),
                    BT::InputPort<bool>("is_sentry_out_of_range", "is sentry out of range?"),
                    BT::InputPort<bool>("is_enemy_out_of_range", "is enemy out of range?"),
                    BT::InputPort<int64_t>("fullKey", "fullKey")
                };
        }

    private:
        rclcpp::Node::SharedPtr node_;
        bool is_enemy_out_of_range;
        bool is_sentry_out_of_range;
        bool is_purple_entry_out_of_range;
        bool is_green_entry_out_of_range;
        example_interfaces::msg::Int64 full_key;
        geometry_msgs::msg::PoseStamped goal_pose;
        rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr full_key_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    };

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_