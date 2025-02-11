#include <string>

#include "robot_behavior_tree/plugins/action/aim_enemy.hpp"


namespace nav2_behavior_tree
{
    //    position_x(0.0),
    //    position_y(0.0)
    AimEnemyAction::AimEnemyAction(
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
    }

    BT::NodeStatus AimEnemyAction::tick()
    {
        if (config().blackboard->get<int>("enemy_num")==0){
            return BT::NodeStatus::FAILURE;
        }
        is_enemy_out_of_range = config().blackboard->get<bool>("is_enemy_out_of_range");
        is_sentry_out_of_range = config().blackboard->get<bool>("is_sentry_out_of_range");
        is_purple_entry_out_of_range = config().blackboard->get<bool>("is_purple_entry_out_of_range");
        is_green_entry_out_of_range = config().blackboard->get<bool>("is_green_entry_out_of_range");
        if (is_enemy_out_of_range==is_sentry_out_of_range){
            // enemy_pose_x=config().blackboard->get<double>("enemy_pose_x");
            // enemy_pose_y=config().blackboard->get<double>("enemy_pose_y");
            // sentry_pose_x=config().blackboard->get<double>("sentry_pose_x");
            // sentry_pose_y=config().blackboard->get<double>("sentry_pose_y");
            
            // if ((enemy_pose_x-sentry_pose_x)*(enemy_pose_x-sentry_pose_x)+(enemy_pose_y-sentry_pose_y)*(enemy_pose_y-sentry_pose_y)<2.0)//过近抑制，防止自己打到自己
            // {
            //     goal_pose.pose.position.x=sentry_pose_x;
            //     goal_pose.pose.position.y=sentry_pose_y;
            // }
            // else{
                goal_pose.pose.position.x=enemy_pose_x;
                goal_pose.pose.position.y=enemy_pose_y;
            // }
            
        }else if(is_sentry_out_of_range==is_purple_entry_out_of_range){
            goal_pose.pose.position.x=config().blackboard->get<double>("purple_entry_pose_x");
            goal_pose.pose.position.y=config().blackboard->get<double>("purple_entry_pose_y");
        }else if(is_sentry_out_of_range==is_green_entry_out_of_range){
            goal_pose.pose.position.x=config().blackboard->get<double>("green_entry_pose_x");
            goal_pose.pose.position.y=config().blackboard->get<double>("green_entry_pose_y");
        }else{
            std::cout<<"error:两个传送门识别错误"<<std::endl;
            return BT::NodeStatus::FAILURE;
        }
        
        goal_pose.header.stamp = node_->now();
        goal_pub_->publish(goal_pose);
        std::cout<<"追击敌人中"<<std::endl;
        return BT::NodeStatus::SUCCESS;
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::AimEnemyAction>("AimEnemy");
}
