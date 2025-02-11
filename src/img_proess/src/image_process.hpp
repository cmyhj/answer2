#ifndef IMAGE_PROCESS_H
#define IMAGE_PROCESS_H

#include <iostream>
#include <thread>
#include <cmath>
#include <array>
#include <random>
#include <queue>
#include <chrono>


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
    ENEMY,
    UNEXPLOREDPOSE
};
enum GameMode{
    EASY,
    HARD
};
enum PixelStatus{
    REACHABLE=0,    
    UNEXPLORED=50,
    OBSTACLE=100
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
    std::vector<std::vector<int>> pixel_status_map;
    bool is_transfering_ = false;
    bool is_bullet_low_=false;
    bool is_completed_explored_=false;
    bool game_mode_;
    bool move_check=true;
    cv::Point2f old_explore_pose;
    std::array<std::array<int, 6>, 7> color_threshold = {};
    std::string map_frame="odom";
    std::string robot_frame="base_link"; 
    cv::Vec3b wallColor;
    int enemy_num_;
    int last_game_start_;
    double sentry_HP_;
    geometry_msgs::msg::Pose2D cmd_vel_pose;
    cv::Point2f shoot_other_enemy_pose;



    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPublisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pubResultCmd;
    rclcpp::Publisher<robot_msgs::msg::MapInfoMsgs>::SharedPtr pubMapInfo;
    rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr pubShoot;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    nav_msgs::msg::Odometry odomAftMapped;  
    rclcpp::TimerBase::SharedPtr move_check_timer_;


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
    void timer_callback();

    // 内联函数：检查是否超出范围
    inline bool isOutOfRange(const cv::Point2f& pose) {
        if (mapInfo[STAR].is_exist_and_out_range==false) {
            return true;
        }
        return pose.x/40 <= mapInfo[STAR].pos.x - 2.5 ||
            pose.x/40 >= mapInfo[STAR].pos.x + 2.5 ||
            (12.8-pose.y/40) <= mapInfo[STAR].pos.y - 1.5 ||
            (12.8-pose.y/40) >= mapInfo[STAR].pos.y + 1.5;
    }
    // 内联函数：检查是否
    inline bool isFarFromSentry(const cv::Point2f& pose) {
        return pose.x/40 <= mapInfo[SENTRY].pos.x -0.6||
            pose.x/40 >= mapInfo[SENTRY].pos.x + 0.6 ||
            (12.8-pose.y/40) <= mapInfo[SENTRY].pos.y - 0.6 ||
            (12.8-pose.y/40) >= mapInfo[SENTRY].pos.y + 0.6;
    }
    inline bool isFarFarFromSentry(const cv::Point2f& pose) {
        return pose.x/40 <= mapInfo[SENTRY].pos.x -3||
            pose.x/40 >= mapInfo[SENTRY].pos.x + 3 ||
            (12.8-pose.y/40) <= mapInfo[SENTRY].pos.y - 3 ||
            (12.8-pose.y/40) >= mapInfo[SENTRY].pos.y + 3;
    }
    inline bool isInSentry(const cv::Point2f& pose) {
        return pose.x/40 >= mapInfo[SENTRY].pos.x - 0.2 &&
            pose.x/40 <= mapInfo[SENTRY].pos.x + 0.2 &&
            (12.8-pose.y/40) >= mapInfo[SENTRY].pos.y - 0.2 &&
            (12.8-pose.y/40) <= mapInfo[SENTRY].pos.y + 0.2;
    }
    // 内联函数：检查是否
    inline bool isInDectorRange(const cv::Point2f& pose) {
        return pose.x/40 >= mapInfo[SENTRY].pos.x - 2 &&
            pose.x/40 <= mapInfo[SENTRY].pos.x + 2 &&
            (12.8-pose.y/40) >= mapInfo[SENTRY].pos.y - 2 &&
            (12.8-pose.y/40) <= mapInfo[SENTRY].pos.y + 2;
    }
    // 内联函数：检查是否远离敌方基地
    inline bool isFarFromEnemyBase(const cv::Point2f& pose) {
        if (mapInfo[ENEMY_BASE].pos.x==1000) {
            return true;
        }
        return pose.x/40 <= mapInfo[ENEMY_BASE].pos.x - 1.0 ||
            pose.x/40 >= mapInfo[ENEMY_BASE].pos.x + 1.0 ||
            (12.8-pose.y/40) <= mapInfo[ENEMY_BASE].pos.y - 1.0 ||
            (12.8-pose.y/40) >= mapInfo[ENEMY_BASE].pos.y + 1.0;
    }

    // 内联函数：计算两点之间的距离
    inline double distance(const cv::Point2f& pos1, const geometry_msgs::msg::Pose2D& pos2) {
        return std::sqrt(std::pow(pos1.x/40 - pos2.x, 2) + std::pow(12.8-pos1.y/40 - pos2.y, 2));
    }

    const int dx[4] = {-1, 0, 0, 1};
    const int dy[4] = {0, 1, -1, 0};

    bool findNearestEplorePose(std::vector<std::vector<int>>& grid, int startX, int startY, int type) {
        int rows = grid.size();
        int cols = grid[0].size();
        if (startX < 0 || startX >= rows || startY < 0 || startY >= cols) {
            return false;
        }
        // 创建一个访问标记数组，防止重复访问
        std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));

        // 创建队列，用于BFS
        std::queue<std::pair<int, int>> q;

        // 将起始点加入队列，并标记为已访问
        q.push({startX, startY});
        visited[startX][startY] = true;

        // BFS遍历
        while (!q.empty()) {
            // 获取当前点
            int x = q.front().first;
            int y = q.front().second;
            q.pop();
            // 处理当前点
            if (grid[x][y] == type) {
                for (int di = -1; di <= 1; ++di) 
                {
                    for (int dj = -1; dj <= 1; ++dj) 
                    {
                        int ni = x + di;
                        int nj = y + dj;
                        if (ni >= 0 && ni < rows && nj >= 0 && nj < cols) 
                        {
                            if (grid[ni][nj]==REACHABLE) 
                            {
                                mapInfo[UNEXPLOREDPOSE].pos.x = x/10.0;
                                mapInfo[UNEXPLOREDPOSE].pos.y = y/10.0;
                                mapInfo[UNEXPLOREDPOSE].is_exist_and_out_range = isOutOfRange(cv::Point2f(x/10.0, 12.8-y/10.0));
                                return true;
                            }
                        }
                    }
                }
            }

            // 遍历四个方向
            for (int i = 0; i < 4; ++i) {
                int newX = x + dx[i];
                int newY = y + dy[i];

                // 检查新点是否在数组范围内且未被访问
                if (newX >= 0 && newX < rows && newY >= 0 && newY < cols && !visited[newX][newY]) {
                    // 将新点加入队列，并标记为已访问
                    q.push({newX, newY});
                    visited[newX][newY] = true;
                }
            }
        }
        return false;
    }
    std::vector<cv::Point> getExtendedPoints(const cv::Point& A, const cv::Point& B) {
        std::vector<cv::Point> points;
        int dx = B.x - A.x;
        int dy = B.y - A.y;

        if (dx == 0 && dy == 0) {
            return points; // A和B重合，无延长线
        }

        int step_x = dx > 0 ? 1 : -1;
        int step_y = dy > 0 ? 1 : -1;

        int abs_dx = abs(dx);
        int abs_dy = abs(dy);

        bool xMajor = abs_dx > abs_dy;
        int error = 0;

        int current_x = B.x;
        int current_y = B.y;

        while (true) {
            // 沿主方向移动
            if (xMajor) {
                current_x += step_x;
                error += abs_dy;
                if (error >= abs_dx) {
                    error -= abs_dx;
                    current_y += step_y;
                }
            } else {
                current_y += step_y;
                error += abs_dx;
                if (error >= abs_dy) {
                    error -= abs_dy;
                    current_x += step_x;
                }
            }

            // 检查是否在图像范围内
            if (current_x < 0 || current_x >= 256 || 
                current_y < 0 || current_y >= 128) {
                break;
            }

            points.push_back(cv::Point(current_x, current_y));
        }

        // 逆序点，从最远到最近
        reverse(points.begin(), points.end());

        return points;
    }

public:
    imgProcess();

};

#endif //ANSWER_ANSWER_H