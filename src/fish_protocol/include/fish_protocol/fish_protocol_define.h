/**
 * @brief 文件描述：待完善
 * @author 小鱼 (fishros@foxmail.com)
 * @version V1.0.0
 * @date 2022-07-24
 * @copyright 版权所有：（鱼香ROS）fishros.org.cn
 */
#ifndef _FISH_PROTOCOL_FISH_PROTOCOL_DEFINE_H_
#define _FISH_PROTOCOL_FISH_PROTOCOL_DEFINE_H_
#include <functional>
#include <iostream>
#include <string>
#include <memory>

#include "fish_protocol/protocol_util.h"

namespace fish_protocol {

class ProtocolConfig {
 public:
  // serial
  int serial_baut_;
  std::string serial_address_;

  ProtocolConfig& operator=(const ProtocolConfig& config) {
    serial_baut_ = config.serial_baut_;
    serial_address_ = config.serial_address_;
    return *this;
  };

 public:
  ProtocolConfig(){};
  ~ProtocolConfig(){};
};

class FishProtocol {
 protected:
  std::function<void(const std::uint8_t*, const std::uint8_t)> recv_uint8_callback;
  ProtocolConfig protocol_config_;

 public:
  FishProtocol(const ProtocolConfig& protocol_config);
  ~FishProtocol(){};

 public:
  virtual int ProtocolSendString(const std::string& data) = 0;
  virtual int ProtocolSenduint8_t(const std::uint8_t* data, const std::uint8_t len) = 0;
  virtual int ProtocolDestory() = 0;
  void SetDataRecvCallback(std::function<void(const std::uint8_t*, const std::uint8_t)> callback);
};



}  // namespace fish_protocol
#endif  // _FISH_PROTOCOL_FISH_PROTOCOL_DEFINE_H_