#include "serial_node.hpp"
using namespace std::chrono_literals;


namespace serial_{

// void serial_node::_initSerial(void)
// {
//     RCLCPP_INFO(serial_node::get_logger(), "串口节点启动中...");
//     protocol_config.serial_baut_ = 115200;
//     protocol_config.serial_address_ = "/dev/agv_serial";
//     serial_pro = std::make_shared<fish_protocol::SerialProtocol>(protocol_config);
//     velo_sub = serial_node::create_subscription<agv_interfaces::msg::AgvVelo>("velo_msgs", 10, 
//     std::bind(&serial_node::velo_callback, this, std::placeholders::_1));
//     timer_ = serial_node::create_wall_timer(5000ms, std::bind(&serial_node::timer_callback, this));
//     //  设置接收到数据后的回调函数
//     serial_pro->SetDataRecvCallback([&](const std::uint8_t* data, const std::uint8_t len) -> void
//     {
//         int i;
//         int cnt = fish_protocol::inverse_frame(rx_buf, data, len, func);
//         if(cnt)
//         {
//             printf("\nreveice: ");
//             for(i = 0; i < len; i++)
//             {
//                 printf("%x ", data[i]);
//             }
//             std::cout << std::endl;
//             serial_node::data_analysis(rx_buf, cnt);
//         }
//     });
// }

// void serial_node::data_analysis(const std::uint8_t* data, const std::uint8_t len)
// {
//     switch (serial_::func)
// 	{
// 		case SET_ENABLE:
// 		case SET_DISENABLE:
// 		case IS_ENABLE:
// 		case CLEAR_FAULT:
// 		case IS_FAULT:
// 		case SET_STOP:
// 		case STOP_TO_ENABLE:
// 			control_analysis(data, len);
// 			break;
// 		case SET_PARAM:
// 		case GET_PARAM:
// 			param_analysis(data, len);
// 			break;
// 		default:
//             printf("I'm here! func:%x\n", serial_::func);
// 			break;
// 	}
// }


// void serial_node::control_analysis(const std::uint8_t* control_retdata, const std::uint8_t len)
// {
//     printf("control_analysis\n");
//     if(len != 4)
//         return ;
//     control_ret.flag = 1;
//     control_ret.func = serial_::func;
//     control_ret.cmd_ret[0] = control_retdata[0];
//     control_ret.cmd_ret[1] = control_retdata[1];
//     control_ret.cmd_ret[2] = control_retdata[2];
//     control_ret.cmd_ret[3] = control_retdata[3];
// }


// void serial_node::param_analysis(const std::uint8_t* param_retdata, const std::uint8_t len)
// {
//     printf("param_analysis\n");
//     switch (serial_::func)
// 	{
// 		case SET_PARAM:
//             if(len != 6)
//                 return ;
//             param_set_ret.flag = 1;
//             param_set_ret.func = serial_::func;
//             param_set_ret.reg.data8[0] = param_retdata[0];
//             param_set_ret.reg.data8[1] = param_retdata[1];
//             param_set_ret.set_ret[0] = param_retdata[2];
//             param_set_ret.set_ret[1] = param_retdata[3];
//             param_set_ret.set_ret[2] = param_retdata[4];
//             param_set_ret.set_ret[3] = param_retdata[5];
// 			break;
// 		case GET_PARAM:
//             if(param_retdata[2])
//             {
//                 if(len != 11)
//                     return ;
//                 param_get16_ret.flag = 1;
//                 param_get16_ret.func = serial_::func;
//                 param_get16_ret.reg.data8[0] = param_retdata[0];
//                 param_get16_ret.reg.data8[1] = param_retdata[1];
//                 for(int i = 0; i < 4; i++)
//                 {
//                     param_get16_ret.get_ret[i].data8[0] = param_retdata[3+i*2];
//                     param_get16_ret.get_ret[i].data8[1] = param_retdata[4+i*2];
//                 }
//             }
//             else
//             {
//                 if(len != 18)
//                     return ;
//                 param_get32_ret.flag = 1;
//                 param_get32_ret.func = serial_::func;
//                 param_get32_ret.reg.data8[0] = param_retdata[0];
//                 param_get32_ret.reg.data8[1] = param_retdata[1];
//                 for(int i = 0; i < 4; i++)
//                 {
//                     param_get32_ret.get_ret[i].data8[0] = param_retdata[3+i*4];
//                     param_get32_ret.get_ret[i].data8[1] = param_retdata[4+i*4];
//                     param_get32_ret.get_ret[i].data8[2] = param_retdata[5+i*4];
//                     param_get32_ret.get_ret[i].data8[3] = param_retdata[6+i*4];
//                 }
//             }
// 			break;
// 		default:
// 			break;
// 	}
// }


// //  发送数组
// void serial_node::senddata(const unsigned char* buf, uint8_t len)
// {
//     serial_pro->ProtocolSenduint8_t(buf, len);
// }
// //  发送字符串
// void serial_node::senddata(const std::string& data)
// {
//     serial_pro->ProtocolSendString(data);
// }

// serial_node::~serial_node(){
//     serial_pro->ProtocolDestory();
// }

// void setenable(bool motor1, bool motor2, bool motor3, bool motor4)
// {
//     uint8_t buf[4] = {0};
//     uint8_t cnt_ = 0;
//     uint8_t func_ = 0x01;
//     std::cout << "setenable test！" << std::endl;
//     if(motor1)
//         buf[cnt_++] = 0x01;
//     if(motor1)
//         buf[cnt_++] = 0x01;
//     if(motor3)
//         buf[cnt_++] = 0x01;
//     if(motor4)
//         buf[cnt_++] = 0x01;
//     int cnt = fish_protocol::frame_packing(buf, tx_buf, cnt_, func_);
//     serial_node::senddata(tx_buf, cnt);
//     for(int i = 0; i < cnt; i++)
//     {
//         printf("%x ", tx_buf[i]);
//     }
//     RCLCPP_INFO(serial_node::get_logger(),"send：%d", cnt);
// }

// void serial_node::velo_callback(const agv_interfaces::msg::AgvVelo::SharedPtr velo_msg)
// {
//     //  回调函数处理
//     uint8_t buf[8] = {0};
//     serial_::union_int16 velo_temp;
//     uint8_t cnt_ = 0;
//     std::cout << "修改速度！" << std::endl;
//     velo_temp.data_int16 = velo_msg->v1.data;
//     buf[cnt_++] = velo_temp.data8[0];
//     buf[cnt_++] = velo_temp.data8[1];
//     velo_temp.data_int16 = velo_msg->v2.data;
//     buf[cnt_++] = velo_temp.data8[0];
//     buf[cnt_++] = velo_temp.data8[1];
//     velo_temp.data_int16 = velo_msg->v3.data;
//     buf[cnt_++] = velo_temp.data8[0];
//     buf[cnt_++] = velo_temp.data8[1];
//     velo_temp.data_int16 = velo_msg->v4.data;
//     buf[cnt_++] = velo_temp.data8[0];
//     buf[cnt_++] = velo_temp.data8[1];
//     int cnt = fish_protocol::frame_packing(buf, tx_buf, cnt_, 0x08);
//     serial_node::senddata(tx_buf, cnt);
//     // this->senddata("hello");
//     RCLCPP_INFO(serial_node::get_logger(),"send：%d", cnt);
// }

// void serial_node::timer_callback(void)
// {
//     uint8_t buf[16] = {0};
//     uint8_t cnt_ = 0;
//     uint8_t func_ = 0x0A;
//     std::cout << "ctrl_cmd test！" << std::endl;
//     if(k%2==0)
//     {
//         // func_ = 0x0A;
//         // buf[cnt_++] = 0x83;// 
//         // buf[cnt_++] = 0x60;// 
//         // func_ = 0x09;
//         buf[cnt_++] = 0x64;
//         buf[cnt_++] = 0x60;
//         // buf[cnt_++] = 0xC8;
//         // buf[cnt_++] = 0x00;
//         // buf[cnt_++] = 0xC8;
//         // buf[cnt_++] = 0x00;
//         // buf[cnt_++] = 0xC8;
//         // buf[cnt_++] = 0x00;
//         // buf[cnt_++] = 0xC8;
//         // buf[cnt_++] = 0x00;
//     }
//     else
//     {
//         func_ = 0x0A;
//         buf[cnt_++] = 0x83;// 
//         buf[cnt_++] = 0x60;// 
//         // func_ = 0x09;
//         // buf[cnt_++] = 0x83;
//         // buf[cnt_++] = 0x60;
//         // buf[cnt_++] = 0xC8;
//         // buf[cnt_++] = 0x00;
//         // buf[cnt_++] = 0xC8;
//         // buf[cnt_++] = 0x00;
//         // buf[cnt_++] = 0xC8;
//         // buf[cnt_++] = 0x00;
//         // buf[cnt_++] = 0xC8;
//         // buf[cnt_++] = 0x00;
//     }   
    
//     // buf[cnt_++] = 0x01;// 
//     // buf[cnt_++] = 0x01;// 
//     // switch (k)
//     // {
//     //     case 0x01:
//     //         func_ = 0x02;
//     //         break;
//     //     case 0x02:
//     //         func_ = 0x01;
//     //         break;
//     //     case 0x03:
//     //         func_ = 0x03;
//     //         break;
//     //     case 0x04:
//     //         func_ = 0x04;
//     //         break;
//     //     case 0x05:
//     //         func_ = 0x05;
//     //         break;
//     //     case 0x06:
//     //         func_ = 0x06;
//     //         break;
//     //     case 0x07:
//     //         func_ = 0x07;
//     //         break;
//     //     default:
//     //         break;
//     // }
//     // k %= 7;
//     k++;
//     // std::cout << "param_cmd test！" << std::endl;
//     // buf[cnt_++] = 0x83;
//     // buf[cnt_++] = 0x60;
//     // buf[cnt_++] = 0xC8;
//     // buf[cnt_++] = 0x00;
//     // buf[cnt_++] = 0xC8;
//     // buf[cnt_++] = 0x00;
//     // buf[cnt_++] = 0xC8;
//     // buf[cnt_++] = 0x00;
//     // buf[cnt_++] = 0xC8;
//     // buf[cnt_++] = 0x00;
//     int cnt = fish_protocol::frame_packing(buf, tx_buf, cnt_, func_);
//     serial_node::senddata(tx_buf, cnt);
//     for(int i = 0; i < cnt; i++)
//     {
//         printf("%x ", tx_buf[i]);
//     }
//     RCLCPP_INFO(serial_node::get_logger(),"send：%d", cnt);
// }


}




int main(int argc, char** argv) {
  /* 初始化rclcpp  */
  rclcpp::init(argc, argv);
  /*产生一个node_01的节点*/
  auto node = std::make_shared<serial_::serial_node>("serial_test");
  /* 运行节点，并检测退出信号 Ctrl+C*/
  rclcpp::spin(node);
  /* 停止运行 */
  rclcpp::shutdown();
  return 0;
}



