#ifndef IF_SHIELD_OPEN_HPP_
#define IF_SHIELD_OPEN_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{
    class IfShieldOpenCondition : public BT::ConditionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param condition_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        IfShieldOpenCondition(
            const std::string &condition_name,
            const BT::NodeConfiguration &conf);

        IfShieldOpenCondition() = delete;

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
                BT::InputPort<bool>("is_enemy_base_exist", "is enemy base shield open?"),
                BT::InputPort<bool>("guess_enemy_base_exist", "guess is enemy base shield open?"),
                BT::OutputPort<double>("health_threshold", "health_threshold"),
            };
        }
    };

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
