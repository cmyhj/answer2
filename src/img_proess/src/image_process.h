#ifndef IMAGE_PROCESS_H
#define IMAGE_PROCESS_H

#include <iostream>
#include <thread>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/msg/image.hpp>
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

enum class TargetType {
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
    geometry_msgs::msg::Point2D pos;
}MapInfo;


class imgProcess : public rclcpp::Node {
private:
    //parameters
    int B_low_threshold_;
    int B_high_threshold_;
    int G_low_threshold_;
    int G_high_threshold_;
    int R_low_threshold_;
    int R_high_threshold_;

    MapInfo mapInfo[7];
    int color_threshold[7][6];
    std::string map_frame="odom";
    std::string robot_frame="base_link"; 
    cv::Vec3b wallColor;


    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPublisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pubResultCmd;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubMapInfo;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    nav_msgs::msg::Odometry odomAftMapped;   

    template<typename T>
    bool is_out_range(cv::Point2f pos);
    void set_posestamp(T & out);
    void set_map_info(const cv::Mat& Image, TargetType type);

    void publish_map(const cv::Mat Image,const cv::Vec3b wallColor);
    void publish_map_info();
    void publish_sentry_odom(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, 
        std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_br);
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void imageCallback(sensor_msgs::msg::Image rosImage);

public:
    imgProcess();

};

#endif //ANSWER_ANSWER_H