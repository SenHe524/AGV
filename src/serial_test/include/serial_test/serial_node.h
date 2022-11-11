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

typedef union 
{
    int16_t data_int16;
    unsigned char data8[2];
}union_int16;


#endif
