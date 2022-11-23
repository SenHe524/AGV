/**
 * @brief 文件描述：待完善
 * @author 小鱼 (fishros@foxmail.com)
 * @version V1.0.0
 * @date 2022-07-24
 * @copyright 版权所有：（鱼香ROS）fishros.org.cn
 */
#ifndef _FISH_PROTOCOL_PROTO_UTIL_H_
#define _FISH_PROTOCOL_PROTO_UTIL_H_

#include <cstdint>
#include <cstdio>
namespace fish_protocol {

#define FIRST_CODE 0x55
#define FIRST_CODE_ 0x57
#define END_CODE 0xBB
#define END_CODE_ 0xBD


/**
 * @brief 截取指定位数据并赋值
 *        小端模式: 高位在前(低内存)，低位在后(高内存)
 *
 */
#define SET_SUB_BYTES1(target, data) target = (data & 0xFF)
#define SET_SUB_BYTES2(target, data) target |= ((data << 8) & 0xFF00)

/**
 * @brief 查表法crc8校验
 *
 * @param buf 输入数组
 * @param len 数组长度
 * @return uint8_t
 */
uint8_t getcrc8tab(const uint8_t *buf, int len);

/**
 * @brief 直接计算法crc8校验
 *
 * @param buf 输入数组
 * @param len 数组长度
 * @return uint8_t
 */
uint8_t getCrc8(const uint8_t *buf, int len);

/**
 * @brief 将数据进行转义
 *
 * @param frame 帧数据
 * @param result 结果
 * @param len 长度
 * @return int 转义完成后新的帧的大小
 */
int frame_packing(const uint8_t *buf, uint8_t *frame, uint8_t len, uint8_t func);

/**
 * @brief 将数据帧进行反转义
 *
 * @param frame 帧数据
 * @param result 结果
 * @param len 长度
 * @return int
 */
int inverse_frame(uint8_t *result, const uint8_t *frame, uint8_t len, uint8_t& func);

/**
 * @brief 将数据帧打印成hex形式
 *
 * @param title 数据标题
 * @param buffer 数据
 * @param size 长度
 */
void print_frame_to_hex(const char* title, const char* buffer, uint16_t size);

}  // namespace fish_protocol

#endif  // _PROTO_UTILS_H_