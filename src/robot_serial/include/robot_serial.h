#ifndef ROBOT_SERIAL_H
#define ROBOT_SERIAL_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "robot_msgs/msg/serial_full_key.hpp"
#include "robot_msgs/msg/serial_segment_key.hpp"

#include "msg_serialize.h"
#include "serialPro.h"
namespace my_serial
{
    message_data Head
    {
        uint64_t SOF =0xAA; //头校验 
        uint64_t length = 0; //长度
        uint64_t cmd_id = 0x00; //命令字
    };
    message_data password_send_t
    {
        uint64_t password1; //密码片段 
        uint64_t password2;
    };
    message_data password_receive_t
    {
        int64_t password; //密码
    };
    message_data Tail
    {
        uint64_t crc16 =0xBB; //尾校验
    };

    class MySerial : public sp::serialPro<Head,Tail>
    {
    public:
        MySerial() = default;
        MySerial(std::string s, int band): sp::serialPro<Head,Tail>(s,band){
            registerChecker([](const Head& head)-> int { return head.SOF != 0xAA; }); //头校验
            registerChecker([](const Tail& tail, const uint8_t*, const int&)-> int { return tail.crc16 != 0xBB; });
            setGetId([](const Head& head)-> int { return head.cmd_id;}); //返回命令字
            setGetLength([](const Head& head)-> int{ return (int)head.length; }); //返回长度
        }
    };
}

class RobotSerial : public rclcpp::Node {
private:
    my_serial::MySerial serial;
    rclcpp::Clock rosClock;
    rclcpp::Publisher<robot_msgs::msg::SerialFullKey>::SharedPtr FullKeyPublisher;
    rclcpp::Subscription<robot_msgs::msg::SerialSegmentKey>::SharedPtr SegmentKeySubscriber;

    void SegmentKeyCallback(const robot_msgs::msg::SerialSegmentKey::SharedPtr msg){
        my_serial::Head head;
        my_serial::Tail tail;
        my_serial::password_send_t password_send{
                (uint64_t)(msg->segment_key_1),
                (uint64_t)(msg->segment_key_2),
        };
        head.length=sizeof(my_serial::password_send_t);
        head.cmd_id=0;
        serial.write(head,password_send,tail);
        RCLCPP_INFO(this->get_logger(),"serial write:%lu,%lu",msg->segment_key_1,msg->segment_key_2);
    }
public:
    explicit RobotSerial() : Node("robot_serial_node"){
        declare_parameter("serial_name", "/dev/pts/3");
        serial =std::move(my_serial::MySerial(get_parameter("serial_name").as_string(),115200));
        RCLCPP_INFO(this->get_logger(),"robot_serial init success");
        serial.registerCallback(1,[this](const my_serial::password_receive_t& msg){
            robot_msgs::msg::SerialFullKey _fullKey;
            _fullKey.full_key=msg.password;
            FullKeyPublisher->publish(_fullKey);
        });
        FullKeyPublisher = create_publisher<robot_msgs::msg::SerialFullKey>("/serial/full_key_",1);
        SegmentKeySubscriber = create_subscription<robot_msgs::msg::SerialSegmentKey>("/serial/segment_key_",1,
                                                                                      std::bind(&RobotSerial::SegmentKeyCallback,this,std::placeholders::_1));
        serial.spin(true);
    }
};
#endif //RDBOT_SERIAL_H