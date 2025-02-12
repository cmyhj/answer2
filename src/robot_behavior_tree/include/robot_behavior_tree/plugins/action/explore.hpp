#ifndef EXPLORE_HPP_
#define EXPLORE_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include <nav2_msgs/action/navigate_to_pose.hpp>

namespace nav2_behavior_tree
{
    class ExploreAction : public BT::SyncActionNode
    {
    public:
        ExploreAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        ExploreAction() = delete;

        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts()
        {
            return {
                    BT::InputPort<double>("explore_pose_x", "Explore position x"),
                    BT::InputPort<double>("explore_pose_y", "Explore position y"),
                    BT::InputPort<double>("purple_entry_pose_x", "Purple entry position x"),
                    BT::InputPort<double>("purple_entry_pose_y", "Purple entry position y"),
                    BT::InputPort<double>("green_entry_pose_x", "Green entry position x"),
                    BT::InputPort<double>("green_entry_pose_y", "Green entry position y"),
                    BT::InputPort<bool>("is_purple_entry_out_of_range", "is purple entry out of range?"),
                    BT::InputPort<bool>("is_green_entry_out_of_range", "is green entry out of range?"),
                    BT::InputPort<bool>("is_sentry_out_of_range", "is sentry out of range?"),
                    BT::InputPort<bool>("is_unexplored_out_of_range", "is unexplored pose _out_of_range?"),
                    BT::InputPort<bool>("is_completed_explored", "is completed explored?"),
                    BT::InputPort<bool>("is_base_exist", "is base exist?"),
                    BT::InputPort<double>("enemy_base_pose_x", "enemy_base_pose_x"),
                };
        }

    private:
        rclcpp::Node::SharedPtr node_;
        bool is_unexplored_out_of_range;
        bool is_sentry_out_of_range;
        bool is_purple_entry_out_of_range;
        bool is_green_entry_out_of_range;
        geometry_msgs::msg::PoseStamped goal_pose;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    };

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
