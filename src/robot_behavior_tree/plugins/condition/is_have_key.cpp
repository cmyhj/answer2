#include <string>

#include "robot_behavior_tree/plugins/condition/is_have_key.hpp"

namespace nav2_behavior_tree
{

    IsHaveKeyCondition::IsHaveKeyCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf)
    {
        enemy_num = config().blackboard->get<int>("enemy_num");
        key_num_ = 0;
        fullKey = 0;
        key_used = (config().blackboard->get<double>("sentry_HP") == 0.0 ? true : false);
        key.segment_key_1 = 0;
        key.segment_key_2 = 0;
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        key_sub_ = node_->create_subscription<example_interfaces::msg::Int64>(
            "/password_segment",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&IsHaveKeyCondition::keyCallback, this, std::placeholders::_1),
            sub_option);
        password_sub_ = node_->create_subscription<robot_msgs::msg::SerialFullKey>(
            "/serial/full_key_",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&IsHaveKeyCondition::passwordCallback, this, std::placeholders::_1),
            sub_option);
        key_pub_ = node_->create_publisher<robot_msgs::msg::SerialSegmentKey>("/serial/segment_key_", 10);
    }

    BT::NodeStatus IsHaveKeyCondition::tick()
    {
        enemy_num = config().blackboard->get<int>("enemy_num");
        key_num_ = config().blackboard->get<int>("key_num_");
        callback_group_executor_.spin_some();
        // std::cout << "key1" << key.segment_key_1
        //           << "key2" << key.segment_key_2
        //           << "password"<<fullKey<< std::endl;
        if (enemy_num == 0)
        {
            auto temp= key.segment_key_1;
            key.segment_key_1 = key.segment_key_2;
            key.segment_key_2 = temp;
            key_pub_->publish(key);
            config().blackboard->set<int64_t>("fullKey", fullKey);
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
    void IsHaveKeyCondition::keyCallback(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        if (key_num_ == 0)
        {
            key.segment_key_1 = msg->data;
        }
        else if (key_num_ == 1)
        {
            key.segment_key_2 = msg->data;
        }
        config().blackboard->set<int>("key_num_",key_num_+1);
    }
    void IsHaveKeyCondition::passwordCallback(const robot_msgs::msg::SerialFullKey::SharedPtr msg)
    {
        fullKey = msg->full_key;
        config().blackboard->set<int64_t>("fullKey", fullKey);
    }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IsHaveKeyCondition>("IsHaveKey");
}
