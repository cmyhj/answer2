#include <nav2_behavior_tree/behavior_tree_engine.hpp>
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/create_timer_ros.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// #include "robot_serial/msg/action.hpp"
// #include "robot_serial/msg/whitelist.hpp"

class DecisionMakerNode : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    DecisionMakerNode(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
        this->declare_parameter("loop_duration_in_millisec", 10);
        this->get_parameter("loop_duration_in_millisec", loop_duration_in_millisec_);
        bt_loop_duration_ = std::chrono::milliseconds(loop_duration_in_millisec_);
        this->declare_parameter("server_timeout_in_millisec", 100);
        this->get_parameter("server_timeout_in_millisec", server_timeout_in_millisec_);
        server_timeout_ = std::chrono::milliseconds(server_timeout_in_millisec_);
        this->declare_parameter("plugin_lib_names", std::vector<std::string>());
        this->get_parameter("plugin_lib_names", plugin_lib_names_);
        this->declare_parameter("bt_xml_filename", std::string(""));
        this->get_parameter("bt_xml_filename", bt_xml_filename_);
        // this->declare_parameter("is_we_are_blue", true);
        // this->get_parameter("is_we_are_blue", is_we_are_blue_);
        // p_pub_ = this->create_publisher<robot_serial::msg::Action>("/robot/action", 1);
        // w_pub_ = this->create_publisher<robot_serial::msg::Whitelist>("/robot/whitelist", 1);
        waitNav2();
        client_node_name_ = this->get_name();
        auto options = rclcpp::NodeOptions().arguments(
            {"--ros-args",
             "-r",
             std::string("__node:=") +
                 client_node_name_ + "_rclcpp_node",
             "--"});
        client_node_ = std::make_shared<rclcpp::Node>("_", options);

        std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("robot_bt_decision_maker");
        bt_xml_filename_ = pkg_share_dir + bt_xml_filename_;
        bt_ = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(plugin_lib_names_);
        tfbuffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            get_node_base_interface(), get_node_timers_interface());
        tfbuffer_->setCreateTimerInterface(timer_interface);
        tfbuffer_->setUsingDedicatedThread(true);
        tflistener_ = std::make_shared<tf2_ros::TransformListener>(*tfbuffer_, this, false);
        blackboard_ = BT::Blackboard::create();
        // Put items on the blackboard
        blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);                    // NOLINT
        blackboard_->set<std::chrono::milliseconds>("server_timeout", server_timeout_);     // NOLINT
        blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_); // NOLINT
        blackboard_->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tfbuffer_);         // NOLINT

        blackboard_->set<double>("star_pose_x", 0.0);
        blackboard_->set<double>("star_pose_y", 0.0);
        blackboard_->set<double>("base_pose_x", 0.0);
        blackboard_->set<double>("base_pose_y", 0.0);
        blackboard_->set<double>("enemy_base_pose_x", 0.0);
        blackboard_->set<double>("enemy_base_pose_y", 0.0);
        blackboard_->set<double>("purple_entry_pose_x", 0.0);
        blackboard_->set<double>("purple_entry_pose_y", 0.0);
        blackboard_->set<double>("green_entry_pose_x", 0.0);
        blackboard_->set<double>("green_entry_pose_y", 0.0);
        blackboard_->set<double>("sentry_pose_x", 0.0);
        blackboard_->set<double>("sentry_pose_y", 0.0);
        blackboard_->set<double>("enemy_pose_x", 0.0);
        blackboard_->set<double>("enemy_pose_y", 0.0);
        blackboard_->set<double>("explore_pose_x", 0.0);
        blackboard_->set<double>("explore_pose_y", 0.0);
        blackboard_->set<int>("enemy_num", 0);
        blackboard_->set<double>("sentry_HP", 0.0);
        blackboard_->set<bool>("is_transfering", false);
        blackboard_->set<bool>("is_star_exist", false);
        blackboard_->set<bool>("is_base_exist", false);
        blackboard_->set<bool>("is_enemy_base_exist", false);
        blackboard_->set<bool>("is_purple_entry_out_of_range", false);
        blackboard_->set<bool>("is_green_entry_out_of_range", false);
        blackboard_->set<bool>("is_sentry_out_of_range", false);
        blackboard_->set<bool>("is_enemy_out_of_range", false);
        blackboard_->set<bool>("is_unexplored_out_of_range", false);
        blackboard_->set<bool>("is_completed_explored", false);
        blackboard_->set<bool>("is_bullet_low", false);

        blackboard_->set<bool>("guess_enemy_base_exist", false);
        blackboard_->set<int>("key_num_", 0);
        blackboard_->set<double>("health_threshold", 95.0);
        blackboard_->set<int64_t>("fullKey",0);
        // blackboard_->set<int>("health_threshold",80);
        if (!loadBehaviorTree(bt_xml_filename_, blackboard_))
        {
            RCLCPP_ERROR(this->get_logger(), "加载行为树失败.");
            return;
        }
        // output_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, 10);
        // input_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_cloud_topic_, 10, std::bind(&ObstacleSegmentationNode::cloudCallback, this, std::placeholders::_1));
        tflistener_ = std::make_shared<tf2_ros::TransformListener>(*tfbuffer_);
    }
    nav2_behavior_tree::BtStatus runBehaviorTree()
    {
        auto is_canceling = [this]() -> bool
        {
            return false;
        };
        auto on_loop = [this]() -> void
        {
            // RCLCPP_INFO(this->get_logger(), "行为树正在运行...");
            rclcpp::spin_some(this->get_node_base_interface());
        };
        // Run the Behavior Tree
        return bt_->run(&tree_, on_loop, is_canceling, bt_loop_duration_);
    }
    void waitNav2()
    {
        std::string node_service = "/bt_navigator/get_state";
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client = this->create_client<lifecycle_msgs::srv::GetState>(node_service);
        while (!client->wait_for_service(std::chrono::seconds(1))){
            RCLCPP_INFO(this->get_logger(), "Waiting for bt_navigator to be available");
        }
    }

private:
    // Parameters
    int loop_duration_in_millisec_;
    int server_timeout_in_millisec_;
    std::vector<std::string> plugin_lib_names_;
    std::string bt_xml_filename_;
    // bool is_we_are_blue_;

    std::unique_ptr<nav2_behavior_tree::BehaviorTreeEngine> bt_;
    BT::Tree tree_;
    BT::Blackboard::Ptr blackboard_;
    std::chrono::milliseconds bt_loop_duration_;
    std::chrono::milliseconds server_timeout_;
    std::string client_node_name_;
    rclcpp::Node::SharedPtr client_node_;
    std::shared_ptr<tf2_ros::Buffer> tfbuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tflistener_;
    geometry_msgs::msg::PoseStamped pose;
    // rclcpp::Publisher<robot_serial::msg::Action>::SharedPtr p_pub_;
    // rclcpp::Publisher<robot_serial::msg::Whitelist>::SharedPtr w_pub_;
    
    bool loadBehaviorTree(const std::string &bt_xml_filename, BT::Blackboard::Ptr blackboard)
    {
        // Read the input BT XML from the specified file into a string
        std::ifstream xml_file(bt_xml_filename);

        if (!xml_file.good())
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't open input XML file: %s", bt_xml_filename.c_str());
            return false;
        }

        //auto xml_string = std::string(
            //std::istreambuf_iterator<char>(xml_file),
           // std::istreambuf_iterator<char>());

        // Create the Behavior Tree from the XML input
        try
        {
            tree_ = bt_->createTreeFromFile(bt_xml_filename, blackboard);
            for (auto &blackboard : tree_.blackboard_stack)
            {
                blackboard->set<rclcpp::Node::SharedPtr>("node", client_node_);                    // NOLINT
                blackboard->set<std::chrono::milliseconds>("server_timeout", server_timeout_);     // NOLINT
                blackboard->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_); // NOLINT
                blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tfbuffer_);         // NOLINT
                // blackboard->set<bool>("is_we_are_blue", is_we_are_blue_);
                blackboard->set<double>("star_pose_x", 0.0);
                blackboard->set<double>("star_pose_y", 0.0);
                blackboard->set<double>("base_pose_x", 0.0);
                blackboard->set<double>("base_pose_y", 0.0);
                blackboard->set<double>("enemy_base_pose_x", 0.0);
                blackboard->set<double>("enemy_base_pose_y", 0.0);
                blackboard->set<double>("purple_entry_pose_x", 0.0);
                blackboard->set<double>("purple_entry_pose_y", 0.0);
                blackboard->set<double>("green_entry_pose_x", 0.0);
                blackboard->set<double>("green_entry_pose_y", 0.0);
                blackboard->set<double>("sentry_pose_x", 0.0);
                blackboard->set<double>("sentry_pose_y", 0.0);
                blackboard->set<double>("enemy_pose_x", 0.0);
                blackboard->set<double>("enemy_pose_y", 0.0);
                blackboard->set<double>("explore_pose_x", 0.0);
                blackboard->set<double>("explore_pose_y", 0.0);
                blackboard->set<int>("enemy_num", 0);
                blackboard->set<double>("sentry_HP", 0.0);
                blackboard->set<bool>("is_transfering", false);
                blackboard->set<bool>("is_star_exist", false);
                blackboard->set<bool>("is_base_exist", false);
                blackboard->set<bool>("is_enemy_base_exist", false);
                blackboard->set<bool>("is_purple_entry_out_of_range", false);
                blackboard->set<bool>("is_green_entry_out_of_range", false);
                blackboard->set<bool>("is_sentry_out_of_range", false);
                blackboard->set<bool>("is_enemy_out_of_range", false);
                blackboard->set<bool>("is_unexplored_out_of_range", false);
                blackboard->set<bool>("is_bullet_low", false);
                blackboard->set<bool>("is_completed_explored", false);

                blackboard->set<bool>("guess_enemy_base_exist", false);
                blackboard->set<double>("health_threshold", 95.0);
                blackboard->set<int>("key_num_", 0);
                blackboard->set<int64_t>("fullKey",0);
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception when loading BT: %s", e.what());
            return false;
        }
        return true;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<DecisionMakerNode>("decision_maker_node");
    /* 运行节点，并检测退出信号*/
    // 当节点没有退出时，循环调用runBehaviorTree
    rclcpp::WallRate loop_rate(100);

    while (rclcpp::ok())
    {
        node->runBehaviorTree();
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}