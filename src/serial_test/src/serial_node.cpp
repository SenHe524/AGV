#include "serial_node.h"
#include "rclcpp/rclcpp.hpp"
using namespace std::chrono_literals;

//  声明占位参数
// using std::placeholders::_1;

uint8_t buf_tx[128] = {0};
uint8_t func = 0;
class serial_node: public rclcpp::Node
{
public:
    serial_node(std::string name):Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "串口节点启动中...");
        protocol_config.serial_baut_ = 115200;
        protocol_config.serial_address_ = "/dev/ttyUSB0";
        serial_pro = std::make_shared<fish_protocol::SerialProtocol>(protocol_config);
        velo_sub = this->create_subscription<agv_interfaces::msg::AgvVelo>("velo_msgs", 10, 
        std::bind(&serial_node::velo_callback, this, std::placeholders::_1));
        //  设置接收到数据后的回调函数
        serial_pro->SetDataRecvCallback([](const std::uint8_t* data, const std::uint8_t len) -> void
        {
            int cnt = fish_protocol::inverse_frame(data, buf_tx, len, func);
            printf("receive：%d, %d\n", len, cnt);
            if(cnt)
            {
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
    rclcpp::Subscription<agv_interfaces::msg::AgvVelo>::SharedPtr velo_sub;

    void velo_callback(const agv_interfaces::msg::AgvVelo::SharedPtr velo_msg)
    {
        //  回调函数处理
        printf("修改速度！\n");
        uint8_t buf[8] = {0};
        velocity_ velo_temp;
        uint8_t cnt_ = 0;
        velo_temp.velo_ = velo_msg->v1.data;
        buf[cnt_++] = velo_temp.data8[0];
        buf[cnt_++] = velo_temp.data8[1];
        velo_temp.velo_ = velo_msg->v2.data;
        buf[cnt_++] = velo_temp.data8[0];
        buf[cnt_++] = velo_temp.data8[1];
        velo_temp.velo_ = velo_msg->v3.data;
        buf[cnt_++] = velo_temp.data8[0];
        buf[cnt_++] = velo_temp.data8[1];
        velo_temp.velo_ = velo_msg->v4.data;
        buf[cnt_++] = velo_temp.data8[0];
        buf[cnt_++] = velo_temp.data8[1];
        int cnt = fish_protocol::frame_packing(buf, buf_tx, cnt_, 0x02);

        this->senddata(buf_tx, cnt);
        // this->senddata("hello");
        RCLCPP_INFO(this->get_logger(),"send：%d", cnt);
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



