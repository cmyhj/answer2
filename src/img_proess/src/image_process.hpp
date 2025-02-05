#ifndef IMAGE_PROCESS_H
#define IMAGE_PROCESS_H

#include <iostream>
#include <thread>
#include <cmath>
#include <array>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <example_interfaces/msg/bool.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "robot_msgs/msg/map_info.hpp"
#include "robot_msgs/msg/map_info_msgs.hpp"


enum TargetType : uint8_t {
    STAR,
    BASE,
    ENEMY_BASE,
    PURPLEENTRY,
    GREENENTRY,
    SENTRY,
    ENEMY
};

typedef struct
{
    TargetType type;
    bool is_exist_and_out_range;
    geometry_msgs::msg::Pose2D pos;
}MapInfo_;


class imgProcess : public rclcpp::Node {
private:
    //parameters
    int B_low_threshold_;
    int B_high_threshold_;
    int G_low_threshold_;
    int G_high_threshold_;
    int R_low_threshold_;
    int R_high_threshold_;

    std::vector<robot_msgs::msg::MapInfo> mapInfo;
    bool is_transfering_ = false;
    std::array<std::array<int, 6>, 7> color_threshold = {};
    std::string map_frame="odom";
    std::string robot_frame="base_link"; 
    cv::Vec3b wallColor;
    int enemy_num_;
    double sentry_HP_;
    geometry_msgs::msg::Pose2D cmd_vel_pose;


    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPublisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pubResultCmd;
    rclcpp::Publisher<robot_msgs::msg::MapInfoMsgs>::SharedPtr pubMapInfo;
    rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr pubShoot;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    nav_msgs::msg::Odometry odomAftMapped;  


    bool is_line_not_cross_obstacle( cv::Mat& image, cv::Point2f pt1, cv::Point2f pt2);
    template<typename T>
    void set_posestamp(T & out);
    void set_map_info(const cv::Mat& Image, uint8_t type);

    void publish_map(const cv::Mat Image,const cv::Vec3b wallColor);
    void publish_map_info();
    void publish_sentry_odom(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, 
        std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_br);
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void imageCallback(sensor_msgs::msg::Image rosImage);

    // 内联函数：检查是否超出范围
    inline bool isOutOfRange(const cv::Point2f& pose) {
        return pose.x <= mapInfo[STAR].pos.x - 2.0 ||
            pose.x >= mapInfo[STAR].pos.x + 2.0 ||
            pose.y <= mapInfo[STAR].pos.y - 1.0 ||
            pose.y >= mapInfo[STAR].pos.y + 1.0;
    }

    // 内联函数：检查是否远离敌方基地
    inline bool isFarFromEnemyBase(const cv::Point2f& pose) {
        return pose.x/40 <= mapInfo[ENEMY_BASE].pos.x - 1.0 ||
            pose.x/40 >= mapInfo[ENEMY_BASE].pos.x + 1.0 ||
            (12.8-pose.y/40) <= mapInfo[ENEMY_BASE].pos.y - 1.0 ||
            (12.8-pose.y/40) >= mapInfo[ENEMY_BASE].pos.y + 1.0;
    }

    // 内联函数：计算两点之间的距离
    inline double distance(const cv::Point2f& pos1, const geometry_msgs::msg::Pose2D& pos2) {
        return std::sqrt(std::pow(pos1.x/40 - pos2.x, 2) + std::pow(12.8-pos1.y/40 - pos2.y, 2));
    }
public:
    imgProcess();

};

#endif //ANSWER_ANSWER_H