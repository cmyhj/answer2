#include "image_process.hpp"
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
    //HPupdate
    int cout = 0;
    for (int i = 0; i <= 380; ++i) {
        int x = i;
        int y = 996;
        cv::Vec3b pixel = img.at<cv::Vec3b>(cv::Point(x, y));
        if (pixel[0] == 131 && pixel[1] == 131 && pixel[2] == 131) {
            cout++;
        }
    }
    sentry_HP_= cout/3.8;
    //bullet update
    cout = 0;
    for (int i = 2032; i <= 2047; ++i) {
        int x = i;
        int y = 1014;
        cv::Vec3b pixel = img.at<cv::Vec3b>(cv::Point(x, y));
        if (pixel[0] >= 200 && pixel[1] >=200 && pixel[2] >=200) {
            cout++;
        }
        y = 1013;
        pixel = img.at<cv::Vec3b>(cv::Point(x, y));
        if (pixel[0] >= 200 && pixel[1] >=200 && pixel[2] >=200) {
            cout++;
        }
    }
    if (cout ==0) {
        is_bullet_low_=true;
    } else {
        is_bullet_low_=false;
    }
    //地图处理
    int newWidth = 256;  // 新的宽度
    int newHeight = 128; // 新的高度
    cv::Mat mapImage,findingImage,viewImage;
    // 使用cv::resize函数降低分辨率
    cv::resize(img, mapImage, cv::Size(newWidth, newHeight), 0, 0, cv::INTER_NEAREST);
    cv::resize(img, findingImage, cv::Size(newWidth*4, newHeight*4), 0, 0, cv::INTER_NEAREST);
    publish_map(mapImage,wallColor);
    for (size_t i = 0; i < 7; i++)
    {
        set_map_info(findingImage,i);
    }
    //发射逻辑
    if( (mapInfo[ENEMY_BASE].is_exist_and_out_range==true
        &&is_line_not_cross_obstacle(findingImage,
                                    cv::Point2f(40*mapInfo[SENTRY].pos.x,-40*(mapInfo[SENTRY].pos.y-12.8)),
                                    cv::Point2f(40*mapInfo[ENEMY_BASE].pos.x,-40*(mapInfo[ENEMY_BASE].pos.y-12.8))
                                    )
        )||(enemy_num_==2
        &&is_line_not_cross_obstacle(findingImage,
                                    cv::Point2f(40*mapInfo[SENTRY].pos.x,-40*(mapInfo[SENTRY].pos.y-12.8)),
                                    shoot_other_enemy_pose
                                    )
        )||(enemy_num_>0
        &&is_line_not_cross_obstacle(findingImage,
                                    cv::Point2f(40*mapInfo[SENTRY].pos.x,-40*(mapInfo[SENTRY].pos.y-12.8)),
                                    cv::Point2f(40*mapInfo[ENEMY].pos.x,-40*(mapInfo[ENEMY].pos.y-12.8))
                                    )//后判断这个，如果都没法发射，机器人瞄准近处的enemy
            )
        )
    {
        example_interfaces::msg::Bool shoot;
        shoot.data = true; // 设置消息内容为 true
        // 发布消息
        pubShoot->publish(shoot);
    }
    // RCLCPP_INFO(this->get_logger(), "----------------sentrypos: %f,%f",mapInfo[SENTRY].pos.x,mapInfo[SENTRY].pos.y);

    //test
    // cv::Mat testImage;
    // cv::inRange(findingImage, cv::Scalar(B_low_threshold_, G_low_threshold_, R_low_threshold_), cv::Scalar(B_high_threshold_, G_high_threshold_, R_high_threshold_), testImage);
    // vector<vector<Point> > test_contours;
    // vector<Vec4i> test_hierarchy;
    // findContours( testImage, test_contours, test_hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
    // vector<vector<Point> > test_contours_poly( test_contours.size() );
    // vector<float>test_radius( test_contours.size() );
    // vector<Point2f>test_centers( test_contours.size() );
    // for( size_t i = 0; i < test_contours.size(); i++ )
    // {
    //     approxPolyDP( test_contours[i], test_contours_poly[i], 3, true );
    //     minEnclosingCircle( test_contours_poly[i], test_centers[i], test_radius[i] );
    //     if(test_radius[i]>4){
    // cv::circle( findingImage,cv::Point2f(40*mapInfo[ENEMY_BASE].pos.x,-40*(mapInfo[ENEMY_BASE].pos.y-12.8)),5,cv::Scalar( 0, 0, 255 ),10);
        // }
    // }
    // 绘制 ENEMY_BASE 区域的框
    // cv::Point2f enemyBaseTopLeft(40*(mapInfo[ENEMY_BASE].pos.x - 1.0), -40*(mapInfo[ENEMY_BASE].pos.y - 1.0-12.8));
    // cv::Point2f enemyBaseBottomRight(40*(mapInfo[ENEMY_BASE].pos.x + 1.0), -40*(mapInfo[ENEMY_BASE].pos.y + 1.0-12.8));
    // cv::rectangle(img, cv::Point(2032,1013), cv::Point(2047,1014), cv::Scalar(0, 0, 255), 2); // 红色框
    // // cv::putText(img, "HP: " + std::to_string(sentry_HP_),cv::Point(10, 950), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
    // cv::imshow("view", img);
    // waitKey(1);
    
    publish_sentry_odom(pubOdomAftMapped, tf_broadcaster);
    publish_map_info();
    pubResultCmd->publish(cmd_vel_pose);
}

void imgProcess::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::random_device rd;  // 随机数种子
    std::mt19937 gen(rd()); // 使用 Mersenne Twister 算法生成随机数
    std::uniform_int_distribution<> dis(0, 9); // 定义随机数范围为 0 到 9
    // 生成随机数
    int random_number = dis(gen);
    if(random_number<1+9*abs(msg->linear.x))
    {
        cmd_vel_pose.x = msg->linear.x;    
    }else{
        // RCLCPP_INFO(this->get_logger(),"减速！random_number: %d,vx:%f",random_number,msg->linear.x);
        cmd_vel_pose.x=0;
    }
    if(random_number<1+9*abs(msg->linear.y)){
        cmd_vel_pose.y = -msg->linear.y;
    }else{
        // RCLCPP_INFO(this->get_logger(),"减速！random_number: %d,vy:%f",random_number,msg->linear.y);
        cmd_vel_pose.y=0;
    }
    
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
    robot_msgs::msg::MapInfoMsgs mapInfoMsgs;
    mapInfoMsgs.map_info = mapInfo;
    mapInfoMsgs.enemy_num = enemy_num_;
    mapInfoMsgs.sentry_hp = sentry_HP_;
    mapInfoMsgs.is_transfering = is_transfering_;
    mapInfoMsgs.is_bullet_low = is_bullet_low_;
    pubMapInfo->publish(mapInfoMsgs);
}
template<typename T>
void imgProcess::set_posestamp(T & out)
{
    out.position.x = mapInfo[SENTRY].pos.x;
    out.position.y = mapInfo[SENTRY].pos.y;
    out.position.z = 0.0;
    out.orientation.x = 0.0;
    out.orientation.y = 0.0;
    out.orientation.z = 0.0;
    out.orientation.w = 1.0;
}

void imgProcess::set_map_info(const cv::Mat& Image, uint8_t type){
    //寻找目标颜色
    cv::Mat binaryImg;
    cv::inRange(Image, cv::Scalar(color_threshold[type][0], color_threshold[type][1], color_threshold[type][2]), 
                       cv::Scalar(color_threshold[type][3], color_threshold[type][4], color_threshold[type][5]), binaryImg);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(binaryImg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );
    //分类讨论
    if(contours.size()==0){
        mapInfo[type].is_exist_and_out_range = false;
        if (type == ENEMY){
            enemy_num_=0;
        }
        if(type == ENEMY_BASE){
            cv::inRange(Image, cv::Scalar(140,110,160), cv::Scalar(160,130,180), binaryImg);
            findContours(binaryImg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );
            vector<vector<Point> > contours_poly( contours.size() );
            vector<float>radius( contours.size() );
            vector<Point2f>centers( contours.size() );
            for( size_t i = 0; i < contours.size(); i++ )
            {
                approxPolyDP( contours[i], contours_poly[i], 3, true );
                minEnclosingCircle( contours_poly[i], centers[i], radius[i] );
            }
            if(contours.size()==1 && radius[0]>6){
                mapInfo[ENEMY_BASE].pos.x = centers[0].x/40;
                mapInfo[ENEMY_BASE].pos.y = 12.8-centers[0].y/40;
            }
        }
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
            if(radius[0]>6){
                mapInfo[type].pos.x = centers[0].x/40;
                mapInfo[type].pos.y = 12.8-centers[0].y/40;
                mapInfo[type].is_exist_and_out_range = true;
            }
            else{
                mapInfo[type].is_exist_and_out_range = false;
            }
            break;
        case SENTRY:
        case GREENENTRY:
        case PURPLEENTRY:
            is_transfering_ = false;
            if(radius[0]>6){
                mapInfo[type].pos.x = centers[0].x/40;
                mapInfo[type].pos.y = 12.8-centers[0].y/40;
                mapInfo[type].is_exist_and_out_range = isOutOfRange(centers[0]);
            }
            else{
                mapInfo[type].is_exist_and_out_range = false;
            }
            break;
        case ENEMY:
            mapInfo[type].is_exist_and_out_range = false;
            enemy_num_=0;
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
            RCLCPP_WARN(this->get_logger(), "more than one object detected,type:%d",type);
            break;
        case GREENENTRY:
        case PURPLEENTRY:
            is_transfering_ = false;
            for( size_t i = 0; i < contours.size(); i++ ){
                if (radius[i]>6){
                    mapInfo[type].pos.x = centers[i].x/40;
                    mapInfo[type].pos.y = 12.8-centers[i].y/40;
                    mapInfo[type].is_exist_and_out_range = isOutOfRange(centers[i]);
                    if (Image.at<cv::Vec3b>(centers[i].y,centers[i].x)==cv::Vec3b(240,170,89)){
                        is_transfering_ = true;
                        RCLCPP_INFO(this->get_logger(),"Transfering");
                    }
                }
                
            }
            break;
        case ENEMY:
        {
            double min_dist = 10000.0;
            enemy_num_=0;
            for( size_t i = 0; i < contours.size(); i++ ){
                if (radius[i]>5 &&
                    isFarFromEnemyBase(centers[i]))
                {
                    enemy_num_+=1;
                    if(enemy_num_==1){
                        shoot_other_enemy_pose=centers[i];
                    }
                    if(distance(centers[i],mapInfo[SENTRY].pos)-isOutOfRange(centers[i])*1000<min_dist)
                    {
                        mapInfo[type].pos.x = centers[i].x/40;
                        mapInfo[type].pos.y = 12.8-centers[i].y/40;
                        mapInfo[type].is_exist_and_out_range = isOutOfRange(centers[i]);
                        min_dist = distance(centers[i],mapInfo[SENTRY].pos)-mapInfo[type].is_exist_and_out_range*1000;
                    }else{
                        shoot_other_enemy_pose=centers[i];
                    }
                }
            }
        }
            break;
        default:
            break;
        }
    }
}

bool imgProcess::is_line_not_cross_obstacle( cv::Mat& image, cv::Point2f pt1, cv::Point2f pt2) {
    // 获取线段的长度
    int dx = pt2.x - pt1.x;
    int dy = pt2.y - pt1.y;
    cmd_vel_pose.theta = atan2(dy, dx);
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
            // 获取像素
            cv::Vec3b pixel = image.at<cv::Vec3b>(cv::Point(x, y));
            // 检查像素值是否满足阈值条件
            if (pixel[0] == wallColor[0] && pixel[1] == wallColor[1] && pixel[2] == wallColor[2]) {
                return false;
            }
        }
    }
    // RCLCPP_ERROR(this->get_logger(), "can shoot!!!!!!!!!!!");
    return true;
}
//构造函数
imgProcess::imgProcess() : Node("img_process_node") {
    RCLCPP_INFO(this->get_logger(), "img_process_node started");
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

    robot_msgs::msg::MapInfo temp;
    temp.is_exist_and_out_range = false;
    temp.pos.x = 0.0;
    temp.pos.y = 0.0;
    temp.pos.theta = 0.0;  // 注意Pose2D包含x,y,theta三个字段
    for(int i=0; i<7; ++i){
        // 添加到vector
        mapInfo.push_back(temp);
    }
    wallColor={58,58,58};
    enemy_num_=0;
    shoot_other_enemy_pose.x=0;
    shoot_other_enemy_pose.y=0;
    cmd_vel_pose.x=0;
    cmd_vel_pose.y=0;
    cmd_vel_pose.theta=0;
    color_threshold[STAR]= {210,110,100,  250,130,130};
    color_threshold[BASE]={80,50,10,  100,70,30};
    color_threshold[ENEMY_BASE]={70,70,140,  80,80,150};
    color_threshold[PURPLEENTRY]={200,90,180,  240,110,220};
    color_threshold[GREENENTRY]={110,180,0,  130,250,70};
    color_threshold[SENTRY]={220, 150, 70,  255, 200, 100};
    color_threshold[ENEMY]={90, 90, 230,  110, 110, 255};
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
    pubMapInfo = this->create_publisher<robot_msgs::msg::MapInfoMsgs>("/map_info", 100000);
    pubShoot = this->create_publisher<example_interfaces::msg::Bool>("/shoot", 1000);
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<imgProcess>());
    if (rclcpp::ok())
        rclcpp::shutdown();
    return 0;
}