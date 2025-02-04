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



class imgProcess : public rclcpp::Node {
private:
    
    nav_msgs::msg::Odometry odomAftMapped;
    cv::Point2f robot_pos;
    cv::Point2f enemy_pos;
    std::string map_frame="odom";
    std::string robot_frame="base_link"; 


    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPublisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pubResultCmd;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubMapInfo;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    

    template<typename T>
    void set_posestamp(T & out);

    void publish_map(const cv::Mat resizedImage,const cv::Vec3b wallColor);
    void publish_map_info();
    void publish_sentry_odom(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, 
        std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_br);
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void imageCallback(sensor_msgs::msg::Image rosImage);

public:
    imgProcess();

};

#endif //ANSWER_ANSWER_H