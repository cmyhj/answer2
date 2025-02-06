#ifndef AIM_ENEMY_HPP_
#define AIM_ENEMY_HPP_

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
    class AimEnemyAction : public BT::SyncActionNode
    {
    public:
        AimEnemyAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        AimEnemyAction() = delete;

        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<int>("enemy_num", "enemy num"),
                BT::InputPort<double>("enemy_pose_x", "Enemy position x"),
                BT::InputPort<double>("enemy_pose_y", "Enemy position y"),
                BT::InputPort<double>("green_entry_pose_x", "Green entry position x"),
                BT::InputPort<double>("green_entry_pose_y", "Green entry position y"),
                BT::InputPort<bool>("is_enemy_out_of_range", "is enemy out of range?"),
                BT::InputPort<bool>("is_green_entry_out_of_range", "is green entry out of range?"),
                BT::InputPort<bool>("is_purple_entry_out_of_range", "is purple entry out of range?"),
                BT::InputPort<bool>("is_sentry_out_of_range", "is sentry out of range?"),
                BT::InputPort<double>("purple_entry_pose_x", "Purple entry position x"),
                BT::InputPort<double>("purple_entry_pose_y", "Purple entry position y"),
                BT::InputPort<double>("sentry_pose_x", "Sentry position x"),
                BT::InputPort<double>("sentry_pose_y", "Sentry position y"),


                };
        }

    private:
        rclcpp::Node::SharedPtr node_;
        bool is_enemy_out_of_range;
        bool is_sentry_out_of_range;
        bool is_purple_entry_out_of_range;
        bool is_green_entry_out_of_range;
        double enemy_pose_x;
        double enemy_pose_y;
        double sentry_pose_x;
        double sentry_pose_y;
        geometry_msgs::msg::PoseStamped goal_pose;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    };

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
