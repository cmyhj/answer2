#include <string>

#include "robot_behavior_tree/plugins/action/go_star.hpp"


namespace nav2_behavior_tree
{
    //    position_x(0.0),
    //    position_y(0.0)
    GoStarAction::GoStarAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        goal_pose.header.stamp = node_->now();
        goal_pose.header.frame_id = "map";
        goal_pose.pose.position.x = 0.0;
        goal_pose.pose.position.y = 0.0;
        goal_pose.pose.position.z = 0.0;
        goal_pose.pose.orientation.x = 0.0;
        goal_pose.pose.orientation.y = 0.0;
        goal_pose.pose.orientation.z = 0.0;
        goal_pose.pose.orientation.w = 1.0;
        full_key.data=0;
        full_key_pub_=node_->create_publisher<example_interfaces::msg::Int64>("/password", 10);
    }

    BT::NodeStatus GoStarAction::tick()
    {
        is_sentry_out_of_range = config().blackboard->get<bool>("is_sentry_out_of_range");
        is_purple_entry_out_of_range = config().blackboard->get<bool>("is_purple_entry_out_of_range");
        is_green_entry_out_of_range = config().blackboard->get<bool>("is_green_entry_out_of_range");
        if (is_sentry_out_of_range==false){
            goal_pose.pose.position.x=config().blackboard->get<double>("star_pose_x");
            goal_pose.pose.position.y=config().blackboard->get<double>("star_pose_y");
            full_key.data=config().blackboard->get<int64_t>("fullKey");
            full_key_pub_->publish(full_key);
            std::cout<<"前往中心星星,发布密码:"<<full_key.data<<std::endl;
        }else if(is_sentry_out_of_range==is_purple_entry_out_of_range){
            goal_pose.pose.position.x=config().blackboard->get<double>("purple_entry_pose_x");
            goal_pose.pose.position.y=config().blackboard->get<double>("purple_entry_pose_y");
            std::cout<<"前往紫色传送门"<<std::endl;
        }else if(is_sentry_out_of_range==is_green_entry_out_of_range){
            goal_pose.pose.position.x=config().blackboard->get<double>("green_entry_pose_x");
            goal_pose.pose.position.y=config().blackboard->get<double>("green_entry_pose_y");
            std::cout<<"前往绿色传送门"<<std::endl;
        }else{
            std::cout<<"error:两个传送门识别错误"<<std::endl;
            return BT::NodeStatus::FAILURE;
        }
        goal_pose.header.stamp = node_->now();
        goal_pub_->publish(goal_pose);
        
        return BT::NodeStatus::SUCCESS;
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::GoStarAction>("GoStar");
}