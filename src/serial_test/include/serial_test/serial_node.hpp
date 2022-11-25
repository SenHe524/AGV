// #ifndef _SERIAL_NODE_H_
// #define _SERIAL_NODE_H_
#pragma once
#include <cstdio>
//  C++处理时间的库
#include <chrono>
#include "fish_protocol/serial_protocol.h"
#include "fish_protocol/protocol_util.h"
#include "rclcpp/rclcpp.hpp"
//  导入消息类型文件
#include "agv_interfaces/msg/agv_velo.hpp"
#include "nav_msgs/msg/odometry.hpp"
namespace serial_ {
using namespace std::chrono_literals;

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

#define SAVE_RW		            0x2009
#define LOCK_METHOD 			0x200F
#define SAVE_RW_S 			    0x2010
#define VELO_SMOOTH_FACTOR 		0x2018
#define ELEC_ERATIO_GAIN 		0x2019
#define ELEC_INTEGRAL 			0x201A
#define FEEDFORWARD_RATIO 		0x201B
#define TORQUE_RATIO 			0x201C
#define VELO_KP 				0x201D
#define VELO_KI 				0x201E
#define VELO_FEEDFORWARD_KF 	0x201F
#define POSI_KP 				0x2020
#define POSI_FEEDFORWARD_KF 	0x2021
#define MOTOR_TEMP 				0x2026
#define IS_MOTOR_MOVE 			0x2027
#define MOTOR_HALL_STATUS		0x2028
#define ERROR_CODE	 			0x603f
#define MOTOR_STATUS			0x6040
#define MOTOR_CONTROL 			0x6041
#define MOTOR_MODE				0x6060
#define MODE_DISPLAY 			0x6061
#define ACTUAL_COUNT			0x6064
#define ACTUAL_VELOCITY			0x606C
#define ACC_TIME	 			0x6083
#define DE_TIME					0x6084
#define TARGET_VELOCITY			0x60FF

#define IS_REG(REG)		        (((REG) == SAVE_RW) || \
                                 ((REG) == LOCK_METHOD) || \
                                 ((REG) == SAVE_RW_S) || \
								 ((REG) == VELO_SMOOTH_FACTOR) || \
								 ((REG) == ELEC_ERATIO_GAIN) || \
								 ((REG) == ELEC_INTEGRAL) || \
								 ((REG) == FEEDFORWARD_RATIO) || \
								 ((REG) == TORQUE_RATIO) || \
								 ((REG) == VELO_KP) || \
								 ((REG) == VELO_KI) || \
								 ((REG) == VELO_FEEDFORWARD_KF) || \
								 ((REG) == POSI_KP) || \
								 ((REG) == POSI_FEEDFORWARD_KF) || \
                                 ((REG) == MOTOR_TEMP) || \
								 ((REG) == IS_MOTOR_MOVE) || \
								 ((REG) == MOTOR_HALL_STATUS) || \
								 ((REG) == ERROR_CODE) || \
                                 ((REG) == MOTOR_STATUS) || \
                                 ((REG) == MOTOR_CONTROL) || \
								 ((REG) == MOTOR_MODE) || \
								 ((REG) == MODE_DISPLAY) || \
                                 ((REG) == ACTUAL_COUNT) || \
								 ((REG) == ACTUAL_VELOCITY) || \
								 ((REG) == ACC_TIME) || \
								 ((REG) == DE_TIME) || \
                                 ((REG) == TARGET_VELOCITY))

#define IS_CAN_SET(REG)		    (((REG) == SAVE_RW) || \
                                 ((REG) == LOCK_METHOD) || \
                                 ((REG) == SAVE_RW_S) || \
								 ((REG) == VELO_SMOOTH_FACTOR) || \
								 ((REG) == ELEC_ERATIO_GAIN) || \
								 ((REG) == ELEC_INTEGRAL) || \
								 ((REG) == FEEDFORWARD_RATIO) || \
								 ((REG) == TORQUE_RATIO) || \
								 ((REG) == VELO_KP) || \
								 ((REG) == VELO_KI) || \
								 ((REG) == VELO_FEEDFORWARD_KF) || \
								 ((REG) == POSI_KP) || \
								 ((REG) == POSI_FEEDFORWARD_KF) || \
								 ((REG) == ACC_TIME) || \
								 ((REG) == DE_TIME))

#define IS_CAN_GET(REG)		    (((REG) != MOTOR_CONTROL) && \
								 ((REG) != MOTOR_MODE))


class serial_node: public rclcpp::Node
{
private:
    serial_protocol::ProtocolConfig protocol_config;  //  串口配置
    std::shared_ptr<serial_protocol::SerialProtocol> serial_pro;//  串口对象
    rclcpp::Subscription<agv_interfaces::msg::AgvVelo>::SharedPtr velo_sub;
    rclcpp::TimerBase::SharedPtr timer_;
    int64_t select = 0x0A;
    int16_t v1 = 0,v2 = 0,v3 = 0,v4 = 0;
    void _initSerial(void);
    void velo_callback(const agv_interfaces::msg::AgvVelo::SharedPtr velo_msg);
    void timer_callback(void);
    void data_analysis(const std::uint8_t* data, const std::uint8_t len);
    void control_analysis(const std::uint8_t* control_retdata, const std::uint8_t len);
    void param_analysis(const std::uint8_t* param_retdata, const std::uint8_t len);

public:
    serial_node(std::string name):Node(name){
        _initSerial();
    }
    ~serial_node()
    {
        serial_pro->ProtocolDestory();
    }

    //  发送数组
    void senddata(const unsigned char* buf, uint8_t len);
    //  发送字符串
    void senddata(const std::string& data);

    
    void control_cmd(uint8_t func);
    void speed_cmd(const int16_t motor1_velo, const int16_t motor2_velo, 
        const int16_t motor3_velo, const int16_t motor4_velo);
    void param_set_cmd(const uint16_t reg, const uint16_t motor1_data, 
        const uint16_t motor2_data, const uint16_t motor3_data, uint16_t motor4_data);
    void param_get_cmd(const uint16_t reg);


    void clearcontrolbuf(void);
    void setenable(void);
    void setdisable(void);
    void isenable(void);
    void clearfault(void);
    void isfault(void);
    void quickstop(void);
    void quickstop_toenable(void);
    void setspeed(const int16_t motor1_velo, const int16_t motor2_velo, 
        const int16_t motor3_velo, const int16_t motor4_velo);

    void clearsetbuf(void);
    void setparam(const uint16_t reg, const uint16_t motor1_data, const uint16_t motor2_data, 
        const uint16_t motor3_data, const uint16_t motor4_data);

    void cleargetbuf(void);
    void getparam(const uint16_t reg);

};


// class serial_node: public rclcpp::Node
// {
// private:
//     serial_protocol::ProtocolConfig protocol_config;  //  串口配置
//     std::shared_ptr<serial_protocol::SerialProtocol> serial_pro;//  串口对象
//     rclcpp::Subscription<agv_interfaces::msg::AgvVelo>::SharedPtr velo_sub;
//     rclcpp::TimerBase::SharedPtr timer_;
//     int64_t select = 0x0A;
//     int16_t v1 = 0,v2 = 0,v3 = 0,v4 = 0;
//     void _initSerial(void)
//     {
//         RCLCPP_INFO(serial_node::get_logger(), "串口节点启动中...");
//         protocol_config.serial_baut_ = 115200;
//         protocol_config.serial_address_ = "/dev/ttyUSB0";
//         serial_pro = std::make_shared<serial_protocol::SerialProtocol>(protocol_config);
//         velo_sub = this->create_subscription<agv_interfaces::msg::AgvVelo>("velo_msgs", 10, 
//         std::bind(&serial_node::velo_callback, this, std::placeholders::_1));
//         timer_ = this->create_wall_timer(5000ms, std::bind(&serial_node::timer_callback, this));
//         this->declare_parameter<std::int64_t>("select", select);
//         //  设置接收到数据后的回调函数
//         serial_pro->SetDataRecvCallback([&](const std::uint8_t* data, const std::uint8_t len) -> void
//         {
//             int cnt = fish_protocol::inverse_frame(rx_buf, data, len, func);
//             if(cnt)
//             {
//                 printf("\nreveice: ");
//                 for(int i = 0; i < len; i++)
//                 {
//                     printf("%x ", data[i]);
//                 }
//                 std::cout << std::endl;
//                 serial_node::data_analysis(rx_buf, cnt);
//             }
//         });
//     }
//     void velo_callback(const agv_interfaces::msg::AgvVelo::SharedPtr velo_msg)
//     {
//         // //  回调函数处理
//         // if( (v1 != velo_msg->v1.data) || 
//         //     (v2 != velo_msg->v2.data) || 
//         //     (v3 != velo_msg->v3.data) || 
//         //     (v4 != velo_msg->v4.data)
//         //     )
//         // {
//         //     v1 = velo_msg->v1.data;
//         //     v2 = velo_msg->v2.data;
//         //     v3 = velo_msg->v3.data;
//         //     v4 = velo_msg->v4.data;
//             speed_cmd(velo_msg->v1.data, velo_msg->v2.data, velo_msg->v3.data, velo_msg->v4.data);
//         // }
//     }
//     void timer_callback(void)
//     {
//         this->get_parameter("select", this->select);
//         if(select == 0)
//         {
//             printf("\n\n");
//             printf("%ld\n",select);
//             setdisable();
//             printf("\n\n");
//         }
//         else if(select == 1)
//         {
//             printf("\n\n");
//             printf("%ld\n",select);
//             isenable();
//             printf("\n\n");
//         }
//         else if(select == 2)
//         {
//             printf("\n\n");
//             printf("%ld\n",select);
//             setenable();
//             printf("\n\n");
//         }
//         else if(select == 3)
//         {
//             // printf("\n\n");
//             // printf("%ld\n",select);
//             // clearfault();
//             // printf("\n\n");
//         }
//         else if(select == 4)
//         {
//             setparam(SAVE_RW_S, 0, 0, 0, 0);
//         }
//         else if(select == 5)
//         {
//             getparam(SAVE_RW_S);
//         }
//         else if(select == 6)
//         {
//             setparam(ACC_TIME, 100, 100, 100, 100);
//         }
//         else if(select == 7)
//         {
//             getparam(ACC_TIME);
//         }
//         else if(select == 8)
//         {

//         }
//         else if(select == 9)
//         {

//         }
//     }
//     void data_analysis(const std::uint8_t* data, const std::uint8_t len){
//         printf("seial_::func = %x\n", serial_::func);
//         switch (serial_::func)
//         {
//             case SET_ENABLE:
//             case SET_DISENABLE:
//             case IS_ENABLE:
//             case CLEAR_FAULT:
//             case IS_FAULT:
//             case SET_STOP:
//             case STOP_TO_ENABLE:
//                 control_analysis(data, len);
//                 break;
//             case SPEED:
//                 break;
//             case SET_PARAM:
//             case GET_PARAM:
//                 param_analysis(data, len);
//                 break;
//             default:
//                 printf("unknown func:%x\n", serial_::func);
//                 break;
//         }
//     }
//     void control_analysis(const std::uint8_t* control_retdata, const std::uint8_t len)
//     {
//         printf("control_analysis\n");
//         if(len != 4)
//             return ;
//         control_ret.flag = 1;
//         control_ret.func = serial_::func;
//         control_ret.cmd_ret[0] = control_retdata[0];
//         control_ret.cmd_ret[1] = control_retdata[1];
//         control_ret.cmd_ret[2] = control_retdata[2];
//         control_ret.cmd_ret[3] = control_retdata[3];
//     }
//     void param_analysis(const std::uint8_t* param_retdata, const std::uint8_t len)
//     {
//         printf("param_analysis\n");
//         switch (serial_::func)
//         {
//             case SET_PARAM:
//                 if(len != 6)
//                     return ;
//                 param_set_ret.flag = 1;
//                 param_set_ret.func = serial_::func;
//                 param_set_ret.reg.data8[0] = param_retdata[0];
//                 param_set_ret.reg.data8[1] = param_retdata[1];
//                 param_set_ret.set_ret[0] = param_retdata[2];
//                 param_set_ret.set_ret[1] = param_retdata[3];
//                 param_set_ret.set_ret[2] = param_retdata[4];
//                 param_set_ret.set_ret[3] = param_retdata[5];
//                 break;
//             case GET_PARAM:
//                 if(param_retdata[2])
//                 {
//                     if(len != 11)
//                         return ;
//                     param_get16_ret.flag = 1;
//                     param_get16_ret.func = serial_::func;
//                     param_get16_ret.reg.data8[0] = param_retdata[0];
//                     param_get16_ret.reg.data8[1] = param_retdata[1];
//                     for(int i = 0; i < 4; i++)
//                     {
//                         param_get16_ret.get_ret[i].data8[0] = param_retdata[3+i*2];
//                         param_get16_ret.get_ret[i].data8[1] = param_retdata[4+i*2];
//                     }
//                 }
//                 else
//                 {
//                     if(len != 18)
//                         return ;
//                     param_get32_ret.flag = 1;
//                     param_get32_ret.func = serial_::func;
//                     param_get32_ret.reg.data8[0] = param_retdata[0];
//                     param_get32_ret.reg.data8[1] = param_retdata[1];
//                     for(int i = 0; i < 4; i++)
//                     {
//                         param_get32_ret.get_ret[i].data8[0] = param_retdata[3+i*4];
//                         param_get32_ret.get_ret[i].data8[1] = param_retdata[4+i*4];
//                         param_get32_ret.get_ret[i].data8[2] = param_retdata[5+i*4];
//                         param_get32_ret.get_ret[i].data8[3] = param_retdata[6+i*4];
//                     }
//                 }
//                 break;
//             default:
//                 break;
//         }
//     }

// public:
//     serial_node(std::string name):Node(name){
//         _initSerial();
//     }
//     ~serial_node()
//     {
//         serial_pro->ProtocolDestory();
//     }

//     //  发送数组
//     void senddata(const unsigned char* buf, uint8_t len)
//     {
//         serial_pro->ProtocolSenduint8_t(buf, len);
//     }
//     //  发送字符串
//     void senddata(const std::string& data)
//     {
//         serial_pro->ProtocolSendString(data);
//     }

    
//     void control_cmd(uint8_t func)
//     {
//         uint8_t buf[4] = {0x01, 0x01, 0x01, 0x01};
//         std::cout << "control_cmd test！" << std::endl;
//         int cnt = fish_protocol::frame_packing(buf, tx_buf, 4, func);
//         this->senddata(tx_buf, cnt);
//         for(int i = 0; i < cnt; i++)
//         {
//             printf("%x ", tx_buf[i]);
//         }
//         std::cout << std::endl;
//         // RCLCPP_INFO(this->get_logger(),"send：%d", cnt);
//     }
//     void speed_cmd(const int16_t motor1_velo, const int16_t motor2_velo, 
//         const int16_t motor3_velo, const int16_t motor4_velo)
//     {
//         union_int16 velo_[4];
//         uint8_t buf[8];
//         std::cout << "speed_cmd test！" << std::endl;
//         velo_[0].data_int16 = motor1_velo;
//         velo_[1].data_int16 = motor2_velo;
//         velo_[2].data_int16 = motor3_velo;
//         velo_[3].data_int16 = motor4_velo;
//         for(int i = 0; i < 4; i++)
//         {
//             buf[i*2] = velo_[i].data8[0];
//             buf[i*2+1] = velo_[i].data8[1];
//         }
//         int cnt = fish_protocol::frame_packing(buf, tx_buf, 8, 0x08);
//         this->senddata(tx_buf, cnt);
//         for(int i = 0; i < cnt; i++)
//         {
//             printf("%x ", tx_buf[i]);
//         }
//         std::cout << std::endl;
//         // RCLCPP_INFO(this->get_logger(),"send：%d", cnt);
//     }
//     void param_set_cmd(const uint16_t reg, const uint16_t motor1_data, 
//         const uint16_t motor2_data, const uint16_t motor3_data, uint16_t motor4_data)
//     {
//         union_uint16 data[4];
//         union_uint16 reg_;
//         uint8_t buf[10];
//         std::cout << "param_set_cmd test！" << std::endl;
//         reg_.data_uint16 = reg;
//         buf[0] = reg_.data8[0];
//         buf[1] = reg_.data8[1];
//         data[0].data_uint16 = motor1_data;
//         data[1].data_uint16 = motor2_data;
//         data[2].data_uint16 = motor3_data;
//         data[3].data_uint16 = motor4_data;
//         for(int i = 0; i < 4; i++)
//         {
//             buf[i*2+2] = data[i].data8[0];
//             buf[i*2+3] = data[i].data8[1];
//         }
//         int cnt = fish_protocol::frame_packing(buf, tx_buf, 10, 0x09);
//         this->senddata(tx_buf, cnt);
//         for(int i = 0; i < cnt; i++)
//         {
//             printf("%x ", tx_buf[i]);
//         }
//         std::cout << std::endl;
//         // RCLCPP_INFO(this->get_logger(),"send：%d", cnt);
//     }
//     void param_get_cmd(const uint16_t reg)
//     {
//         union_uint16 reg_;
//         uint8_t buf[2];
//         std::cout << "param_get_cmd test！" << std::endl;
//         reg_.data_uint16 = reg;
//         buf[0] = reg_.data8[0];
//         buf[1] = reg_.data8[1];
//         int cnt = fish_protocol::frame_packing(buf, tx_buf, 2, 0x0A);
//         this->senddata(tx_buf, cnt);
//         for(int i = 0; i < cnt; i++)
//         {
//             printf("%x ", tx_buf[i]);
//         }
//         std::cout << std::endl;
//         // RCLCPP_INFO(this->get_logger(),"send：%d", cnt);
//     }


//     void clearcontrolbuf(void)
//     {
//         control_ret.flag = 0;
//         control_ret.func = 0;
//         for(int i = 0; i < 4; i++)
//         {
//             control_ret.cmd_ret[i] = 0;
//         }
//     }
//     void setenable(void)
//     {
//         // clearcontrolbuf();
//         control_cmd(0x01);
//     }
//     void setdisable(void)
//     {
//         // clearcontrolbuf();
//         control_cmd(0x02);
//     }
//     void isenable(void)
//     {
//         // clearcontrolbuf();
//         control_cmd(0x03);
//     }
//     void clearfault(void)
//     {
//         // clearcontrolbuf();
//         control_cmd(0x04);
//     }
//     void isfault(void)
//     {
//         // clearcontrolbuf();
//         control_cmd(0x05);
//     }
//     void quickstop(void)
//     {
//         // clearcontrolbuf();
//         control_cmd(0x06);
//     }
//     void quickstop_toenable(void)
//     {
//         // clearcontrolbuf();
//         control_cmd(0x07);
//     }
//     void setspeed(const int16_t motor1_velo, const int16_t motor2_velo, 
//         const int16_t motor3_velo, const int16_t motor4_velo)
//     {
//         speed_cmd(motor1_velo, motor2_velo, motor3_velo, motor4_velo);
//     }

//     void clearsetbuf(void)
//     {
//         param_set_ret.flag = 0;
//         param_set_ret.func = 0;
//         param_set_ret.reg.data_uint16 = 0;
//         for(int i = 0; i < 4; i++)
//         {
//             control_ret.cmd_ret[i] = 0;
//         }
//     }
//     void setparam(const uint16_t reg, const uint16_t motor1_data, const uint16_t motor2_data, 
//         const uint16_t motor3_data, const uint16_t motor4_data)
//     {
//         // clearsetbuf();
//         if(IS_CAN_SET(reg)){
//             param_set_cmd(reg, motor1_data, motor2_data, motor3_data, motor4_data);
//         }
//         else{
//             printf("当前参数无法设置！\n");
//         }
//     }

//     void cleargetbuf(void)
//     {
//         param_get16_ret.flag = 0;
//         param_get16_ret.func = 0;
//         param_get16_ret.reg.data_uint16 = 0;
//         param_get32_ret.flag = 0;
//         param_get32_ret.func = 0;
//         param_get32_ret.reg.data_uint16 = 0;
//         for(int i = 0; i < 4; i++)
//         {
//             param_get16_ret.get_ret[i].data_uint16 = 0;
//             param_get32_ret.get_ret[i].data_int32 = 0;
//         }
//     }
//     void getparam(const uint16_t reg)
//     {
//         if(IS_CAN_GET(reg))
//         {
//             cleargetbuf();
//             param_get_cmd(reg);
//         }
//         else{
//             printf("当前参数无法获取！\n");
//         }
//     }

// };

}

// #endif
