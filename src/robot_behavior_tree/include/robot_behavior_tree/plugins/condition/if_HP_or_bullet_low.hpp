#ifndef IF_HP_OR_BULLET_LOW_HPP_
#define IF_HP_OR_BULLET_LOW_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{
    class IfHPOrBulletLowCondition : public BT::ConditionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param condition_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        IfHPOrBulletLowCondition(
            const std::string &condition_name,
            const BT::NodeConfiguration &conf);

        IfHPOrBulletLowCondition() = delete;

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
                BT::InputPort<double>("sentry_HP", "Sentry health points"),
                BT::InputPort<bool>("is_bullet_low", "is_bullet_low"),
            };
        }

    private:
        double sentry_HP_;
        double HP_threshold_;
        bool is_HP_low_;
    };

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
