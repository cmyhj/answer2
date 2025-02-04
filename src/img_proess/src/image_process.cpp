#include "image_process.h"

using namespace std::chrono_literals;
using namespace cv;
using namespace std;

void imgProcess::imageCallback(sensor_msgs::msg::Image rosImage) {
    auto cvImage = cv_bridge::toCvCopy(rosImage, rosImage.encoding);
    cv::Mat img = cvImage->image;
    cvtColor(img, img, CV_RGB2BGR);
    
    //地图发布
    auto wallColor = img.at<cv::Vec3b>(cv::Point(img.cols-1,1));
    int newWidth = 256;  // 新的宽度
    int newHeight = 128; // 新的高度
    cv::Mat mapImage,findingImage,viewImage;
    // 使用cv::resize函数降低分辨率
    cv::resize(img, mapImage, cv::Size(newWidth, newHeight), 0, 0, cv::INTER_NEAREST);
    cv::resize(img, findingImage, cv::Size(newWidth*4, newHeight*4), 0, 0, cv::INTER_NEAREST);
    cv::resize(img, viewImage,cv::Size(newWidth*4, newHeight*4), 0, 0, cv::INTER_NEAREST);
    //cv::circle( img,cv::Point(2040,1),8,cv::Scalar( 0, 0, 255 ),cv::FILLED,cv::LINE_8);
    // cv::imshow("view", resizedImage);
    // cv::waitKey(1);
    publish_map(mapImage,wallColor);
    //哨兵tf发布
    cv::Mat sentryImage,enemyImage;
    cv::inRange(findingImage, cv::Scalar(220, 150, 60), cv::Scalar(255, 200, 100), sentryImage);
    cv::inRange(findingImage, cv::Scalar(0, 0, 200), cv::Scalar(200, 200, 255), enemyImage);

    //cv::medianBlur(sentryImage, sentryImage, 3);

    Mat enemy_canny_output;
    Mat sentry_canny_output;
    Canny( enemyImage, enemy_canny_output, 255, 255*2 );
    Canny( sentryImage, sentry_canny_output, 255, 255*2 );
 
    vector<vector<Point> > enemy_contours;
    vector<Vec4i> enemy_hierarchy;
    vector<vector<Point> > sentry_contours;
    vector<Vec4i> sentry_hierarchy;

    findContours( enemy_canny_output, enemy_contours, enemy_hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
    findContours( sentry_canny_output, sentry_contours, sentry_hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );

    vector<vector<Point> > contours_poly( sentry_contours.size() );
    vector<float>radius( sentry_contours.size() );
    vector<Point2f>centers( sentry_contours.size() );
    for( size_t i = 0; i < sentry_contours.size(); i++ )
    {
        approxPolyDP( sentry_contours[i], contours_poly[i], 3, true );
        minEnclosingCircle( contours_poly[i], centers[i], radius[i] );
        if(radius[i]>3){
            cv::circle( viewImage,centers[i],int(radius[i]),cv::Scalar( 0, 0, 255 ),1);
            robot_pos=centers[i]/40;
            robot_pos.y=12.8-robot_pos.y;
        }
    }
    
    vector<vector<Point> > enemy_contours_poly( enemy_contours.size() );
    vector<float>enemy_radius( enemy_contours.size() );
    vector<Point2f>enemy_centers( enemy_contours.size() );
    for( size_t i = 0; i < enemy_contours.size(); i++ )
    {
        approxPolyDP( enemy_contours[i], enemy_contours_poly[i], 3, true );
        minEnclosingCircle( enemy_contours_poly[i], enemy_centers[i], enemy_radius[i] );
        if(radius[i]>3){
            cv::circle( viewImage,enemy_centers[i],int(enemy_radius[i]),cv::Scalar( 0, 0, 255 ),1);
            enemy_pos=enemy_centers[i]/40;
            enemy_pos.y=12.8-enemy_pos.y;
        }
    }
    cv::imshow("view", viewImage);
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

//构造函数
imgProcess::imgProcess() : Node("img_process_node") {
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