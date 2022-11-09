#ifndef _SERIAL_NODE_H_
#define _SERIAL_NODE_H_
#include <cstdio>
//  C++处理时间的库
#include <chrono>
#include "fish_protocol/fish_protocol_define.h"
#include "fish_protocol/serial_protocol.h"
#include "fish_protocol/protocol_util.h"



//  导入消息类型文件
#include "agv_interfaces/msg/agv_velo.hpp"

typedef union velocity_
{
    int16_t velo_;
    unsigned char data8[2];
}velocity_;

// std_msgs/Int16 v1
// std_msgs/Int16 v2
// std_msgs/Int16 v3
// std_msgs/Int16 v4

#endif
