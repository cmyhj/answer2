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
                    BT::InputPort<double>("enemy_pose_x", "Enemy position x"),
                    BT::InputPort<double>("enemy_pose_y", "Enemy position y"),
                };
        }

    private:
        rclcpp::Node::SharedPtr node_;
        geometry_msgs::msg::PoseStamped enemy_pose;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    };

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
