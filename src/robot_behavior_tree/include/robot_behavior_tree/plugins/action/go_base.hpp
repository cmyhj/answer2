#ifndef GO_BASE_HPP_
#define GO_BASE_HPP_

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
    class GoBaseAction : public BT::SyncActionNode
    {
    public:
        GoBaseAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        GoBaseAction() = delete;

        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts()
        {
            return {
                    BT::InputPort<double>("base_pose_x", "Base position x"),
                    BT::InputPort<double>("base_pose_y", "Base position y"),
                };
        }

    private:
        rclcpp::Node::SharedPtr node_;
        geometry_msgs::msg::PoseStamped base_pose_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    };

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
