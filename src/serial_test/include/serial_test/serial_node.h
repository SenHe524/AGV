#ifndef _SERIAL_NODE_H_
#define _SERIAL_NODE_H_
#include <cstdio>
//  C++处理时间的库
#include <chrono>
#include "fish_protocol/fish_protocol_define.h"
#include "fish_protocol/serial_protocol.h"
#include "fish_protocol/protocol_util.h"
#include "rclcpp/rclcpp.hpp"
//  导入消息类型文件
#include "agv_interfaces/msg/agv_velo.hpp"

namespace serial_ {

typedef union 
{
    uint16_t data_uint16;
    unsigned char data8[2];
}union_uint16;

typedef union 
{
    int16_t data_int16;
    unsigned char data8[2];
}union_int16;

typedef union 
{
    int32_t data_int32;
    unsigned char data8[4];
}union_int32;

//  控制指令返回数据
typedef struct
{
    uint8_t flag;
    uint8_t func;
    uint8_t cmd_ret[4];
}control_;
control_ control_ret;

//  设置参数指令返回数据
typedef struct
{
    union_uint16 reg;
    uint8_t flag;
    uint8_t func;
    uint8_t set_ret[4];
}param_set;
param_set param_set_ret;

//  获取16位参数指令返回数据
typedef struct
{
    union_uint16 reg;
    uint8_t flag;
    uint8_t func;
    union_uint16 get_ret[4];
}param_get16;
param_get16 param_get16_ret;

//  获取32位参数指令返回数据
typedef struct
{
    union_uint16 reg;
    uint8_t flag;
    uint8_t func;
    union_int32 get_ret[4];
}param_get32;
param_get32 param_get32_ret;

int k = 1;
uint8_t tx_buf[32] = {0};
uint8_t rx_buf[32] = {0};
uint8_t func = 0;

#define SET_ENABLE				0x01
#define SET_DISENABLE			0x02
#define IS_ENABLE				0x03
#define CLEAR_FAULT				0x04
#define IS_FAULT				0x05
#define SET_STOP				0x06
#define STOP_TO_ENABLE			0x07
#define SPEED					0x08
#define SET_PARAM				0x09
#define GET_PARAM				0x0A


class serial_node: public rclcpp::Node
{
private:
    fish_protocol::ProtocolConfig protocol_config;  //  串口配置
    std::shared_ptr<fish_protocol::SerialProtocol> serial_pro;//  串口对象
    rclcpp::Subscription<agv_interfaces::msg::AgvVelo>::SharedPtr velo_sub;
    rclcpp::TimerBase::SharedPtr timer_;
    void velo_callback(const agv_interfaces::msg::AgvVelo::SharedPtr velo_msg);
    void timer_callback(void);
    void _initSerial(void);
    void data_analysis(const std::uint8_t* data, const std::uint8_t len);
    void control_analysis(const std::uint8_t* data, const std::uint8_t len);
    void param_analysis(const std::uint8_t* data, const std::uint8_t len);

public:
    serial_node(std::string name):Node(name){
        _initSerial();
    };
    ~serial_node();
    //  发送数组
    void senddata(const unsigned char* buf, uint8_t len);
    //  发送字符串
    void senddata(const std::string& data);
    //  control——帧
    void control_frame();


};

}

#endif
