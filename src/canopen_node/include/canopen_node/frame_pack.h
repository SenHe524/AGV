#ifndef _FRAME_PACK_H_
#define _FRAME_PACK_H_

#include <cstdint>
#include <cstdio>
namespace frame_pack {

#define FIRST_CODE 0x55
#define END_CODE 0xBB


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
size_t frame_packing(const uint8_t *buf, uint8_t *frame, uint8_t len, uint8_t func);

/**
 * @brief 将数据帧进行反转义
 *
 * @param frame 帧数据
 * @param result 结果
 * @param len 长度
 * @return int
 */
size_t inverse_frame(uint8_t *result, const uint8_t *frame, uint8_t len, uint8_t& func);


}

#endif  // _PROTO_UTILS_H_