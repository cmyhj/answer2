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
                BT::OutputPort<double>("enemy_pose_x","Enemy position x"),
                BT::OutputPort<double>("enemy_pose_y","Enemy position y"),
                };
        }

    private:
        void mapInfoCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        geometry_msgs::msg::PoseStamped enemy_pose;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr enemy_pose_sub_;
    };

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
