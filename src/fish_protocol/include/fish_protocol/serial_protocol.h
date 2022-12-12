/**
 * @brief 文件描述：待完善
 * @author 小鱼 (fishros@foxmail.com)
 * @version V1.0.0
 * @date 2022-07-24
 * @copyright 版权所有：（鱼香ROS）fishros.org.cn
 */
#ifndef _FISH_PROTOCOL_SERIAL_PROTOCOL_H_
#define _FISH_PROTOCOL_SERIAL_PROTOCOL_H_
#include <unistd.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <string>

namespace serial_protocol{

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

enum flush_type
{
  flush_receive = TCIFLUSH,
  flush_send = TCOFLUSH,
  flush_both = TCIOFLUSH
};

class SerialProtocol{
    using spb = boost::asio::serial_port_base;
protected:
    std::function<void(const std::uint8_t*, const std::uint8_t)> recv_uint8_callback;
    ProtocolConfig protocol_config_;
private:
    void _asyncReadSomeData();
    void _initSerialProtocol();
    void _recvDataCallback(const boost::system::error_code& error,
                        size_t bytes_transferred);
    void flush_serial_port(
            boost::asio::serial_port& serial_port,
            flush_type what,
            boost::system::error_code& error);
public:
    SerialProtocol(const ProtocolConfig& protocol_config)
        : io_service_(),
        serial_port_(io_service_) {
        this->protocol_config_ = protocol_config;
        _initSerialProtocol();
    }
    ~SerialProtocol();
    
    int ProtocolSendString(const std::string& data);
    int ProtocolSenduint8_t(const std::uint8_t* data, const std::uint8_t len);
    void SetDataRecvCallback(std::function<void(const std::uint8_t*, const std::uint8_t)> callback);
    // int ProtocolDestory(void);
private:
    bool isfirst = true;
    unsigned char recv_data_buffer_[1024] = {0};
    boost::asio::io_service io_service_;
    boost::asio::serial_port serial_port_;
};
}  // namespace fish_protocol
#endif  // _FISH_PROTOCOL_SERIAL_PROTOCOL_H_
