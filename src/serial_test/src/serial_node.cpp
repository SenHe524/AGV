// #include <cstdio>
// //  C++处理时间的库
// #include <chrono>
// // #include "fish_protocol/fish_protocol_define.h"
// // #include "fish_protocol/serial_protocol.h"
// // #include "fish_protocol/protocol_util.h"
// // #include "fish_protocol/fish_protocol.h"
// // #include "rclcpp/rclcpp.hpp"
// #include "fish_protocol/fish_protocol.h"
// #include "rclcpp/rclcpp.hpp"
// using namespace std::chrono_literals;

// //  声明占位参数
// using std::placeholders::_1;


// class serial_node: public rclcpp::Node
// {

// public:
//     serial_node(std::string name):Node(name)
//     {
//         RCLCPP_INFO(this->get_logger(), "serial节点已经启动.");
//         protocol_config.protocol_type_ = fish_protocol::PROTOCOL_TYPE::SERIAL;
//         protocol_config.serial_baut_ = 115200;
//         protocol_config.serial_address_ = "/dev/ttyUSB0";
//         serial_pro = std::make_shared<fish_protocol::FishProtocol>(protocol_config);
//         timer_ = this->create_wall_timer(1000ms, std::bind(&serial_node::timer_callback, this));
//         serial_pro->SetDataRecvCallback([](const std::uint8_t* data, const std::uint8_t len) -> void
//         {
//             for(int i = 0; i < len; i++)
//             {
//                 std::cout << data[i] << " ";
//             }
//         });
//     }
//     int senddata(const unsigned char* buf, uint8_t len)
//     {
//         serial_pro->ProtocolSendRawData(buf, len);
//         return 0;
//     }
//     // void receivestring(void)
//     // {
//     //     serial_pro->SetDataRecvCallback([](const std::string& data) -> void
//     //     {
//     //         std::cout << "recv" << data << std::endl;
//     //     });
//     // }

//     ~serial_node(){
//         serial_pro->ProtocolDestory();
//     }

// private:
//     fish_protocol::ProtocolConfig protocol_config;  //  配置串口
//     std::shared_ptr<fish_protocol::FishProtocol> serial_pro;//  串口对象
//     rclcpp::TimerBase::SharedPtr timer_;

//     void timer_callback()
//     {
//         //  回调函数处理
//         uint8_t buf[11] = {0x55,0x01,0x06,0x01,0x02,0x00,0x00,0x00,0x00,0x03,0xBB};
//         this->senddata(buf, 11);
//         RCLCPP_INFO(this->get_logger(),"test!\n");
//     }
// };




// int main(int argc, char** argv) {
//   /* 初始化rclcpp  */
//   rclcpp::init(argc, argv);
//   /*产生一个node_01的节点*/
//   auto node = std::make_shared<serial_node>("serial_test");

//   /* 运行节点，并检测退出信号 Ctrl+C*/
//   rclcpp::spin(node);
//   /* 停止运行 */
//   rclcpp::shutdown();
//   return 0;
// }

// #include "/usr/local/include/fish_protocol/fish_protocol.h"
// #include "rclcpp/rclcpp.hpp"
// uint8_t buf[11] = {0x55,0x01,0x06,0x01,0x02,0x00,0x00,0x00,0x00,0x03,0xBB};

// int main(int argc, char** argv) {
//   /* 初始化rclcpp  */
//   rclcpp::init(argc, argv);
//   /*产生一个node_01的节点*/
//   auto node = std::make_shared<rclcpp::Node>("example_fish_protocol");
//   // 打印一句自我介绍
//   RCLCPP_INFO(node->get_logger(), "example_fish_protocol节点已经启动.");

//   fish_protocol::ProtocolConfig proto_config;
//   proto_config.protocol_type_ = fish_protocol::PROTOCOL_TYPE::SERIAL;
//   proto_config.serial_baut_ = 115200;
//   proto_config.serial_address_ = "/dev/ttyUSB0";
//   // 初始化
//   auto protocol = GetProtocolByConfig(proto_config);
//   // 发送数据
//   protocol->ProtocolSendRawData(buf, 11);
//   // 接收数据
//   protocol->SetDataRecvCallback([](const std::uint8_t* data, const std::uint8_t len) -> void
// {
//     for(int i = 0; i < len; i++)
//     {
//         std::cout << data[i] << " ";
//     }
// });
//   // 销毁
//   protocol->ProtocolDestory();

//   /* 运行节点，并检测退出信号 Ctrl+C*/
//   rclcpp::spin(node);
//   /* 停止运行 */
//   rclcpp::shutdown();
//   return 0;
// }


#include <cstdio>
//  C++处理时间的库
#include <chrono>
#include "fish_protocol/fish_protocol_define.h"
#include "fish_protocol/serial_protocol.h"
#include "fish_protocol/protocol_util.h"
#include "rclcpp/rclcpp.hpp"
using namespace std::chrono_literals;

uint8_t buf_tx[128] = {0};
uint8_t func = 0;
class serial_node: public rclcpp::Node
{

public:
    serial_node(std::string name):Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "serial节点已经启动.");
        protocol_config.serial_baut_ = 115200;
        protocol_config.serial_address_ = "/dev/ttyUSB0";
        serial_pro = std::make_shared<fish_protocol::SerialProtocol>(protocol_config);
        timer_ = this->create_wall_timer(1000ms, std::bind(&serial_node::timer_callback, this));
        //  设置接收到数据后的回调函数
        serial_pro->SetDataRecvCallback([](const std::uint8_t* data, const std::uint8_t len) -> void
        {
            int cnt = fish_protocol::inverse_frame(data, buf_tx, len, func);
            if(cnt)
            {
                printf("receive：%d, %d\n", len, cnt);
                for(int i = 0; i < cnt; i++)
                {
                    // std::cout << data[i];
                    printf("%x ", buf_tx[i]);
                }
                std::cout << std::endl;
                for(int i = 0; i < len; i++)
                {
                    // std::cout << data[i];
                    printf("%x ", data[i]);
                }
                std::cout << std::endl;
            }
            
    
        });
    }
    //  发送数组
    void senddata(const unsigned char* buf, uint8_t len)
    {
        serial_pro->ProtocolSenduint8_t(buf, len);
    }
    //  发送字符串
    void senddata(const std::string& data)
    {
        serial_pro->ProtocolSendString(data);
    }

    ~serial_node(){
        serial_pro->ProtocolDestory();
    }

private:
    fish_protocol::ProtocolConfig protocol_config;  //  串口配置
    std::shared_ptr<fish_protocol::SerialProtocol> serial_pro;//  串口对象
    rclcpp::TimerBase::SharedPtr timer_;//  定时器

    void timer_callback()
    {
        //  回调函数处理
        uint8_t buf[6] = {0x01,0x04,0x05,0x07,0x04,0x09};
        int cnt = fish_protocol::frame_packing(buf, buf_tx, 6, 0x01);

        this->senddata(buf_tx, cnt);
        // this->senddata("hello");
        RCLCPP_INFO(this->get_logger(),"send：%d\n", cnt);
    }
};




int main(int argc, char** argv) {
  /* 初始化rclcpp  */
  rclcpp::init(argc, argv);
  /*产生一个node_01的节点*/
  auto node = std::make_shared<serial_node>("serial_test");

  /* 运行节点，并检测退出信号 Ctrl+C*/
  rclcpp::spin(node);
  /* 停止运行 */
  rclcpp::shutdown();
  return 0;
}



