#include "image_process.h"
#define IS_OUT_RANGE(pos) \
    ((pos).x <= MapInfo[STAR].pos.x - 2.0 || \
     (pos).x >= MapInfo[STAR].pos.x + 2.0 || \
     (pos).y <= MapInfo[STAR].pos.y - 1.0 || \
     (pos).y >= MapInfo[STAR].pos.y + 1.0)
#define IS_FAR_FROM_ENEMY_BASE(pos) \
    ((pos).x <= MapInfo[ENEMY_BASE].pos.x - 2.0 || \
     (pos).x >= MapInfo[ENEMY_BASE].pos.x + 2.0 || \
     (pos).y <= MapInfo[ENEMY_BASE].pos.y - 1.0 || \
     (pos).y >= MapInfo[ENEMY_BASE].pos.y + 1.0)
#define DISTANCE(pos1,pos2) \
    sqrt(pow((pos1).x-(pos2).x,2)+pow((pos1).y-(pos2).y,2))
using namespace std::chrono_literals;
using namespace cv;
using namespace std;

void imgProcess::imageCallback(sensor_msgs::msg::Image rosImage) {
    this->get_parameter("B_low_threshold", B_low_threshold_);
    this->get_parameter("G_low_threshold", G_low_threshold_);
    this->get_parameter("R_low_threshold", R_low_threshold_);
    this->get_parameter("B_high_threshold", B_high_threshold_);
    this->get_parameter("G_high_threshold", G_high_threshold_);
    this->get_parameter("R_high_threshold", R_high_threshold_);
    auto cvImage = cv_bridge::toCvCopy(rosImage, rosImage.encoding);
    cv::Mat img = cvImage->image;
    cvtColor(img, img, CV_RGB2BGR);
    
    //地图发布
    int newWidth = 256;  // 新的宽度
    int newHeight = 128; // 新的高度
    cv::Mat mapImage,findingImage,viewImage;
    // 使用cv::resize函数降低分辨率
    cv::resize(img, mapImage, cv::Size(newWidth, newHeight), 0, 0, cv::INTER_NEAREST);
    cv::resize(img, findingImage, cv::Size(newWidth*4, newHeight*4), 0, 0, cv::INTER_NEAREST);
    publish_map(mapImage,wallColor);
    for (size_t i = 0; i < 7; i++)
    {
        set_map_info(findingImage,i)
    }
    

    //test
    cv::Mat testImage;
    cv::inRange(findingImage, cv::Scalar(B_low_threshold_, G_low_threshold_, R_low_threshold_), cv::Scalar(B_high_threshold_, G_high_threshold_, R_high_threshold_), testImage);
    vector<vector<Point> > test_contours;
    vector<Vec4i> test_hierarchy;
    findContours( testImage, test_contours, test_hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
    vector<vector<Point> > test_contours_poly( test_contours.size() );
    vector<float>test_radius( test_contours.size() );
    vector<Point2f>test_centers( test_contours.size() );
    for( size_t i = 0; i < test_contours.size(); i++ )
    {
        approxPolyDP( test_contours[i], test_contours_poly[i], 3, true );
        minEnclosingCircle( test_contours_poly[i], test_centers[i], test_radius[i] );
        if(radius[i]>3){
            cv::circle( findingImage,test_centers[i],int(test_radius[i]),cv::Scalar( 0, 0, 255 ),1);
        }
    }
    cv::imshow("view", findingImage);
    waitKey(1);

    publish_sentry_odom(pubOdomAftMapped, tf_broadcaster);
    publish_map_info();
}

void imgProcess::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    geometry_msgs::msg::Pose2D cmd_vel_pose;
    cmd_vel_pose.x = msg->linear.x;
    cmd_vel_pose.y = -msg->linear.y;
    cmd_vel_pose.theta = msg->angular.z;
    pubResultCmd->publish(cmd_vel_pose);
}

void imgProcess::publish_map(const cv::Mat resizedImage,const cv::Vec3b wallColor){
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id="map";
    map.header.stamp = this->get_clock()->now();
    map.info.resolution = 0.1;         // float32
    map.info.width      = 256;           // uint32
    map.info.height     = 128;           // uint32
    map.info.origin.position.x = 0.0;
    map.info.origin.position.y = 0.0;
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.x = 0.0;
    map.info.origin.orientation.y = 0.0;
    map.info.origin.orientation.z = 0.0;
    map.info.origin.orientation.w = 1.0;
    map.data.resize(map.info.width * map.info.height);
    for (int i = 0; i < resizedImage.rows; ++i) {
        for (int j = 0; j < resizedImage.cols; ++j) {
            cv::Vec3b pixel = resizedImage.at<cv::Vec3b>(cv::Point(j,resizedImage.rows-i));
            int index = i * resizedImage.cols + j;
            if (pixel[0] == wallColor[0] && pixel[1] == wallColor[1] && pixel[2] == wallColor[2]) {
                map.data[index] = 100;
            } else {
                map.data[index] = 0;
            }
        }
    }
    mapPublisher->publish(map);
    // RCLCPP_INFO(this->get_logger(), "map published");
}
void imgProcess::publish_sentry_odom(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_br)
{
    odomAftMapped.header.frame_id = map_frame;
    odomAftMapped.child_frame_id = robot_frame;
    odomAftMapped.header.stamp = this->get_clock()->now();
    set_posestamp(odomAftMapped.pose.pose);
    pubOdomAftMapped->publish(odomAftMapped);
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.transform.translation.x = odomAftMapped.pose.pose.position.x;
    transformStamped.transform.translation.y = odomAftMapped.pose.pose.position.y;
    transformStamped.transform.translation.z = odomAftMapped.pose.pose.position.z;
    transformStamped.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    transformStamped.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    transformStamped.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    transformStamped.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    transformStamped.header.stamp = rclcpp::Time(odomAftMapped.header.stamp);
    transformStamped.header.frame_id = map_frame;
    transformStamped.child_frame_id = robot_frame;
    // std::cout << "a"<<std::endl;
    tf_br->sendTransform(transformStamped);
}
void imgProcess::publish_map_info(){
    geometry_msgs::msg::PoseStamped enemy_pose;
    enemy_pose.header.stamp = this->get_clock()->now();
    enemy_pose.header.frame_id = "map";
    enemy_pose.pose.position.x = enemy_pos.x;
    enemy_pose.pose.position.y = enemy_pos.y;
    enemy_pose.pose.position.z = 0.0;
    enemy_pose.pose.orientation.x = 0.0;
    enemy_pose.pose.orientation.y = 0.0;
    enemy_pose.pose.orientation.z = 0.0;
    enemy_pose.pose.orientation.w = 1.0;
    pubMapInfo->publish(enemy_pose);
}
template<typename T>
void imgProcess::set_posestamp(T & out)
{
    out.position.x = robot_pos.x;
    out.position.y = robot_pos.y;
    out.position.z = 0.0;
    out.orientation.x = 0.0;
    out.orientation.y = 0.0;
    out.orientation.z = 0.0;
    out.orientation.w = 1.0;
}

void imgProcess::set_map_info(const cv::Mat& Image, TargetType type){
    //寻找目标颜色
    cv::Mat binaryImg;
    cv::inRange(Image, cv::Scalar(color_threshold[type][0], color_threshold[type][1], color_threshold[type][2]), 
                       cv::Scalar(color_threshold[type][3], color_threshold[type][4], color_threshold[type][5]), binaryImg);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(binaryImg, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
    //分类讨论
    if(contours.size()==0){
        MapInfo[type].is_exist_and_out_range = false;
        return;
    }
    // 寻找最小外接圆
    vector<vector<Point> > contours_poly( contours.size() );
    vector<float>radius( contours.size() );
    vector<Point2f>centers( contours.size() );
    for( size_t i = 0; i < contours.size(); i++ )
    {
        approxPolyDP( contours[i], contours_poly[i], 3, true );
        minEnclosingCircle( contours_poly[i], centers[i], radius[i] );
    }
    if (contours.size() == 1) {
        switch (type)
        {
        case STAR:
        case BASE:
        case ENEMY_BASE:
            if(radius[i]>3){
                MapInfo[type].pos.x = centers[i].x/40;
                MapInfo[type].pos.y = 12.8-centers[i].y/40;
                MapInfo[type].is_exist_and_out_range = true;
            }
            else{
                MapInfo[type].is_exist_and_out_range = false;
            }
            break;
        case SENTRY:
        case GREENENTRY:
        case PURPLEENTRY:
            if(radius[i]>3){
                MapInfo[type].pos.x = centers[i].x/40;
                MapInfo[type].pos.y = 12.8-centers[i].y/40;
                MapInfo[type].is_exist_and_out_range = IS_OUT_RANGE(centers[i]);
            }
            else{
                MapInfo[type].is_exist_and_out_range = false;
            }
            break;
        case ENEMY:
            MapInfo[type].is_exist_and_out_range = false;
            break;
        default:
            break;
        }
    }
    else{
        switch (type)
        {
        case STAR:
        case BASE:
        case ENEMY_BASE:
        case SENTRY:
        case GREENENTRY:
        case PURPLEENTRY:
           RCLCPP_ERROR(this->get_logger(), "more than one object detected,type:%d",type);
            break;
        case ENEMY:
            double min_dist = 1000.0;
            for( size_t i = 0; i < contours.size(); i++ ){
                if (radius[i]>3 &&
                    IS_FAR_FROM_ENEMY_BASE(centers[i])&&
                    IS_OUT_RANGE(cenrters[i])&&
                    DISTANCE(centers[i],mapInfo[SENTRY].pos)<min_dist)
                {
                    min_dist = DISTANCE(centers[i],mapInfo[SENTRY].pos);
                    MapInfo[type].pos.x = centers[i].x/40;
                    MapInfo[type].pos.y = 12.8-centers[i].y/40;
                    MapInfo[type].is_exist_and_out_range = true;
                }
                else{
                    MapInfo[type].is_exist_and_out_range = false;
                }
            }
            break;
        default:
            break;
        }
    }
}

bool imgProcess::is_line_not_cross_obstacle(const cv::Mat& image, cv::Point2f pt1, cv::Point2f pt2) {
    // 获取线段的长度
    int dx = pt2.x - pt1.x;
    int dy = pt2.y - pt1.y;
    int steps = std::max(abs(dx), abs(dy));

    // 计算步长
    float xIncrement = static_cast<float>(dx) / steps;
    float yIncrement = static_cast<float>(dy) / steps;

    // 遍历线段上的每个像素
    for (int i = 0; i <= steps; ++i) {
        int x = static_cast<int>(pt1.x + i * xIncrement);
        int y = static_cast<int>(pt1.y + i * yIncrement);

        // 检查像素是否在图像范围内
        if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
            // 获取像素值
            cv::Vec3b pixel = image.at<cv::Vec3b>(cv::Point(y, x));
            // 检查像素值是否满足阈值条件
            if (pixel[0] == wallColor[0] && pixel[1] == wallColor[1] && pixel[2] == wallColor[2]) {
                return false
            }
        }
    }
    return true;
}
//构造函数
imgProcess::imgProcess() : Node("img_process_node") {
    RCLCPP_INFO(this->get_logger(), "img_process_node started");`
    this->declare_parameter("B_low_threshold", 0);
    this->declare_parameter("G_low_threshold", 0);
    this->declare_parameter("R_low_threshold", 0);
    this->declare_parameter("B_high_threshold", 0);
    this->declare_parameter("G_high_threshold", 0);
    this->declare_parameter("R_high_threshold", 0);

    this->get_parameter("B_low_threshold", B_low_threshold_);
    this->get_parameter("G_low_threshold", G_low_threshold_);
    this->get_parameter("R_low_threshold", R_low_threshold_);
    this->get_parameter("B_high_threshold", B_high_threshold_);
    this->get_parameter("G_high_threshold", G_high_threshold_);
    this->get_parameter("R_high_threshold", R_high_threshold_);
    wallColor = img.at<cv::Vec3b>(cv::Point(img.cols-1,1));
    RCLCPP_WARN(this->get_logger(), "*-*-*--*********wall color: %d %d %d", wallColor[0], wallColor[1], wallColor[2]);
    mapInfo[STAR].type = STAR;
    mapInfo[BASE].type = BASE;
    mapInfo[ENEMY_BASE].type = ENEMY_BASE;
    mapInfo[PURPLEENTRY].type = PURPLEENTRY;
    mapInfo[GREENENTRY].type = GREENENTRY;
    mapInfo[ENTRY].type = ENTRY;
    mapInfo[SENTRY].type = SENTRY;
    mapInfo[ENEMY].type = ENEMY;
    MapInfo[ENEMY+1].type = ENEMY;
    for (size_t  i = 0; i < 8; i++)
    {
        mapInfo[i].pos.x = 0.0;
        mapInfo[i].pos.y = 0.0;
        mapInfo[i].is_exist_and_out_range = false;
    }
    
    color_threshold[STAR]= {0,0,0,  0,0,0};
    color_threshold[BASE]={0,0,0,  0,0,0};
    color_threshold[ENEMY_BASE]={0,0,0,  0,0,0};
    color_threshold[PURPLEENTRY]={0,0,0,  0,0,0};
    color_threshold[GREENENTRY]={0,0,0,  0,0,0};
    color_threshold[SENTRY]={220, 150, 60,  255, 200, 100};
    color_threshold[ENEMY]={0, 0, 200,  200, 200, 255};
    // 创建QoS配置
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);  // 设置可靠性策略为Reliable
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);  // 设置持久性策略为Transient Local
    //图像订阅
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
            "/image_raw",
            10000,
            std::bind(&imgProcess::imageCallback, this, std::placeholders::_1)
    );
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&imgProcess::cmd_vel_callback, this, std::placeholders::_1));
    cv::namedWindow("view", cv::WINDOW_AUTOSIZE);
    //map发布
    mapPublisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map",qos);
    pubOdomAftMapped = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 100000);
    pubResultCmd = this->create_publisher<geometry_msgs::msg::Pose2D>("/pose", 100000);
    pubMapInfo = this->create_publisher<geometry_msgs::msg::PoseStamped>("/map_info", 100000);
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    robot_pos=cv::Point2f(0.0,0.0);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<imgProcess>());
    if (rclcpp::ok())
        rclcpp::shutdown();
    return 0;
}