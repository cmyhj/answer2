#ifndef IS_HAVE_KEY_HPP_
#define IS_HAVE_KEY_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include <example_interfaces/msg/int64.hpp>
#include "robot_msgs/msg/serial_full_key.hpp"
#include "robot_msgs/msg/serial_segment_key.hpp"

namespace nav2_behavior_tree
{
    class IsHaveKeyCondition : public BT::ConditionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param condition_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        IsHaveKeyCondition(
            const std::string &condition_name,
            const BT::NodeConfiguration &conf);

        IsHaveKeyCondition() = delete;

        /**
         * @brief The main override required by a BT action
         * @return BT::NodeStatus Status of tick execution
         */
        BT::NodeStatus tick() override;

        /**
         * @brief Creates list of BT ports
         * @return BT::PortsList Containing node-specific ports
         */
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<int>("enemy_num", "Number of enemies"),
                BT::InputPort<int>("key_num_", "key_num_"),
                BT::OutputPort<int>("enemy_num", "Number of enemies"),
                BT::OutputPort<int>("key_num_", "key_num_"),
                BT::OutputPort<double>("health_threshold", "health_threshold"),
                BT::OutputPort<int64_t>("fullKey", "fullKey"),
            };
        }

    private:
        bool key_used;
        robot_msgs::msg::SerialSegmentKey key;
        int64_t fullKey;
        int key_num_;
        int enemy_num;
        void keyCallback(const example_interfaces::msg::Int64::SharedPtr msg);
        void passwordCallback(const robot_msgs::msg::SerialFullKey::SharedPtr msg);
        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr key_sub_;
        rclcpp::Subscription<robot_msgs::msg::SerialFullKey>::SharedPtr password_sub_;
        rclcpp::Publisher<robot_msgs::msg::SerialSegmentKey>::SharedPtr key_pub_;
    };

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
