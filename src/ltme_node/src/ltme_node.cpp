#include "ltme_node/ltme_node.h"
#include <arpa/inet.h>

#include <sensor_msgs/msg/laser_scan.hpp>

const std::string LidarDriver::DEFAULT_ENFORCED_TRANSPORT_MODE = "none";
const std::string LidarDriver::DEFAULT_FRAME_ID = "laser";
const int LidarDriver::DEFAULT_SCAN_FREQUENCY = 15;
const double LidarDriver::ANGLE_MIN_LIMIT = -2.356;
const double LidarDriver::ANGLE_MAX_LIMIT = 2.356;
const double LidarDriver::DEFAULT_ANGLE_EXCLUDED_MIN = -3.142;
const double LidarDriver::DEFAULT_ANGLE_EXCLUDED_MAX = -3.142;
const double LidarDriver::RANGE_MIN_LIMIT = 0.05;
const double LidarDriver::RANGE_MAX_LIMIT = 30;
const int LidarDriver::DEFAULT_AVERAGE_FACTOR = 1;
const int LidarDriver::DEFAULT_SHADOW_FILTER_STRENGTH = 50;
const int LidarDriver::DEFAULT_RECEIVER_SENSITIVITY_BOOST = 0;

LidarDriver::LidarDriver()
  : node_(rclcpp::Node::make_shared("ltme_node"))
  , hibernation_requested_(false)
  , quit_driver_(false)
{
  RCLCPP_INFO(node_->get_logger(), "ltme_node started");

  node_->declare_parameter("device_model");
  node_->declare_parameter("device_address");
  node_->declare_parameter("enforced_transport_mode");
  node_->declare_parameter("frame_id");
  node_->declare_parameter("scan_frequency_override");
  node_->declare_parameter("angle_min");
  node_->declare_parameter("angle_max");
  node_->declare_parameter("angle_excluded_min");
  node_->declare_parameter("angle_excluded_max");
  node_->declare_parameter("range_min");
  node_->declare_parameter("range_max");
  node_->declare_parameter("average_factor");
  node_->declare_parameter("shadow_filter_strength");
  node_->declare_parameter("receiver_sensitivity_boost");

  if (!node_->get_parameter("device_model", device_model_)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required parameter \"device_model\"");
    exit(-1);
  }
  if (!node_->get_parameter("device_address", device_address_)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required parameter \"device_address\"");
    exit(-1);
  }
  node_->get_parameter_or("enforced_transport_mode", enforced_transport_mode_, DEFAULT_ENFORCED_TRANSPORT_MODE);
  node_->get_parameter_or("frame_id", frame_id_, DEFAULT_FRAME_ID);
  node_->get_parameter_or("scan_frequency_override", scan_frequency_override_, 0);
  node_->get_parameter_or("angle_min", angle_min_, ANGLE_MIN_LIMIT);
  node_->get_parameter_or("angle_max", angle_max_, ANGLE_MAX_LIMIT);
  node_->get_parameter_or("angle_excluded_min", angle_excluded_min_, DEFAULT_ANGLE_EXCLUDED_MIN);
  node_->get_parameter_or("angle_excluded_max", angle_excluded_max_, DEFAULT_ANGLE_EXCLUDED_MAX);
  node_->get_parameter_or("range_min", range_min_, RANGE_MIN_LIMIT);
  node_->get_parameter_or("range_max", range_max_, RANGE_MAX_LIMIT);
  node_->get_parameter_or("average_factor", average_factor_, DEFAULT_AVERAGE_FACTOR);
  node_->get_parameter_or("shadow_filter_strength", shadow_filter_strength_, DEFAULT_SHADOW_FILTER_STRENGTH);
  node_->get_parameter_or("receiver_sensitivity_boost", receiver_sensitivity_boost_, DEFAULT_RECEIVER_SENSITIVITY_BOOST);

  if (!(enforced_transport_mode_ == "none" || enforced_transport_mode_ == "normal" || enforced_transport_mode_ == "oob")) {
    RCLCPP_ERROR(node_->get_logger(), "Transport mode \"%s\" not supported", enforced_transport_mode_.c_str());
    exit(-1);
  }
  if (scan_frequency_override_ != 0 &&
    (scan_frequency_override_ < 10 || scan_frequency_override_ > 30 || scan_frequency_override_ % 5 != 0)) {
    RCLCPP_ERROR(node_->get_logger(), "Scan frequency %d not supported", scan_frequency_override_);
    exit(-1);
  }
  if (!(angle_min_ < angle_max_)) {
    RCLCPP_ERROR(node_->get_logger(), "angle_min (%f) can't be larger than or equal to angle_max (%f)", angle_min_, angle_max_);
    exit(-1);
  }
  if (angle_min_ < ANGLE_MIN_LIMIT) {
    RCLCPP_ERROR(node_->get_logger(), "angle_min is set to %f while its minimum allowed value is %f", angle_min_, ANGLE_MIN_LIMIT);
    exit(-1);
  }
  if (angle_max_ > ANGLE_MAX_LIMIT) {
    RCLCPP_ERROR(node_->get_logger(), "angle_max is set to %f while its maximum allowed value is %f", angle_max_, ANGLE_MAX_LIMIT);
    exit(-1);
  }
  if (!(range_min_ < range_max_)) {
    RCLCPP_ERROR(node_->get_logger(), "range_min (%f) can't be larger than or equal to range_max (%f)", range_min_, range_max_);
    exit(-1);
  }
  if (range_min_ < RANGE_MIN_LIMIT) {
    RCLCPP_ERROR(node_->get_logger(), "range_min is set to %f while its minimum allowed value is %f", range_min_, RANGE_MIN_LIMIT);
    exit(-1);
  }
  if (range_max_ > RANGE_MAX_LIMIT) {
    RCLCPP_ERROR(node_->get_logger(), "range_max is set to %f while its maximum allowed value is %f", range_max_, RANGE_MAX_LIMIT);
    exit(-1);
  }
  if (average_factor_ <= 0 || average_factor_ > 8) {
    RCLCPP_ERROR(node_->get_logger(), "average_factor is set to %d while its valid value is between 1 and 8", average_factor_);
    exit(-1);
  }
  if (shadow_filter_strength_ < 0 || average_factor_ > 100) {
    RCLCPP_ERROR(node_->get_logger(), "shadow_filter_strength is set to %d while its valid value is between 0 and 100 (inclusive)", shadow_filter_strength_);
    exit(-1);
  }
  if (receiver_sensitivity_boost_ < -20 || receiver_sensitivity_boost_ > 10) {
    RCLCPP_ERROR(node_->get_logger(), "receiver_sensitivity_boost is set to %d while the valid range is between -20 and 10 (inclusive)", receiver_sensitivity_boost_);
    exit(-1);
  }
}

void LidarDriver::run()
{
  std::unique_lock<std::mutex> lock(mutex_);
  
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher = node_->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);//rclcpp::SensorDataQoS()
  rclcpp::Service<agv_interfaces::srv::QuerySerial>::SharedPtr query_serial_service = node_->create_service<agv_interfaces::srv::QuerySerial>
    ("query_serial", std::bind(&LidarDriver::querySerialService, this, std::placeholders::_1, std::placeholders::_2));
  rclcpp::Service<agv_interfaces::srv::QueryFirmwareVersion>::SharedPtr query_firmware_service = node_->create_service<agv_interfaces::srv::QueryFirmwareVersion>
    ("query_firmware_version", std::bind(&LidarDriver::queryFirmwareVersion, this, std::placeholders::_1, std::placeholders::_2));
  rclcpp::Service<agv_interfaces::srv::QueryHardwareVersion>::SharedPtr query_hardware_service = node_->create_service<agv_interfaces::srv::QueryHardwareVersion>
    ("query_hardware_version", std::bind(&LidarDriver::queryHardwareVersion, this, std::placeholders::_1, std::placeholders::_2));
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr request_hibernation_service = node_->create_service<std_srvs::srv::Empty>
    ("request_hibernation", std::bind(&LidarDriver::requestHibernationService, this, std::placeholders::_1, std::placeholders::_2));
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr request_wake_up_service = node_->create_service<std_srvs::srv::Empty>
    ("request_wake_up", std::bind(&LidarDriver::requestWakeUpService, this, std::placeholders::_1, std::placeholders::_2));
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr quit_driver_service = node_->create_service<std_srvs::srv::Empty>
    ("quit_driver", std::bind(&LidarDriver::quitDriverService, this, std::placeholders::_1, std::placeholders::_2));

  using rclcpp::executors::MultiThreadedExecutor;
  MultiThreadedExecutor executor;
  executor.add_node(node_);

  std::string address_str = device_address_;
  std::string port_str = "2105";

  size_t position = device_address_.find(':');
  if (position != std::string::npos) {
    address_str = device_address_.substr(0, position);
    port_str = device_address_.substr(position + 1);
  }

  in_addr_t address = htonl(INADDR_NONE);
  in_port_t port = 0;
  try {
    address = inet_addr(address_str.c_str());
    if (address == htonl(INADDR_NONE))
      throw std::exception();
    port = htons(std::stoi(port_str));
  }
  catch (...) {
    RCLCPP_ERROR(node_->get_logger(), "Invalid device address: %s", device_address_.c_str());
    exit(-1);
  }

  ldcp_sdk::NetworkLocation location(address, port);
  device_ = std::unique_ptr<ldcp_sdk::Device>(new ldcp_sdk::Device(location));

  rclcpp::Rate loop_rate(0.3);
  while (rclcpp::ok() && !quit_driver_.load()) {
    if (device_->open() == ldcp_sdk::no_error) {
      hibernation_requested_ = false;

      lock.unlock();

      RCLCPP_INFO(node_->get_logger(), "Device opened");

      bool reboot_required = false;
      if (enforced_transport_mode_ != "none") {
        std::string firmware_version;
        if (device_->queryFirmwareVersion(firmware_version) == ldcp_sdk::no_error) {
          if (firmware_version < "0201")
            RCLCPP_WARN(node_->get_logger(), "Firmware version %s supports normal transport mode only, "
              "\"enforced_transport_mode\" parameter will be ignored", firmware_version.c_str());
          else {
            bool oob_enabled = false;
            if (device_->isOobEnabled(oob_enabled) == ldcp_sdk::no_error) {
              if ((enforced_transport_mode_ == "normal" && oob_enabled) ||
                  (enforced_transport_mode_ == "oob" && !oob_enabled)) {
                RCLCPP_INFO(node_->get_logger(), "Transport mode will be switched to \"%s\"", oob_enabled ? "normal" : "oob");
                device_->setOobEnabled(!oob_enabled);
                device_->persistSettings();
                reboot_required = true;
              }
            }
            else
              RCLCPP_WARN(node_->get_logger(), "Unable to query device for its current transport mode, "
                "\"enforced_transport_mode\" parameter will be ignored");
          }
        }
        else
          RCLCPP_WARN(node_->get_logger(), "Unable to query device for firmware version, \"enforced_transport_mode\" parameter will be ignored");
      }

      if (!reboot_required) {
        int scan_frequency = DEFAULT_SCAN_FREQUENCY;
        if (scan_frequency_override_ != 0)
          scan_frequency = scan_frequency_override_;
        else {
          if (device_->getScanFrequency(scan_frequency) != ldcp_sdk::no_error)
            RCLCPP_WARN(node_->get_logger(), "Unable to query device for scan frequency and will use %d as the frequency value", scan_frequency);
        }

        if (shadow_filter_strength_ != DEFAULT_SHADOW_FILTER_STRENGTH) {
          if (device_->setShadowFilterStrength(shadow_filter_strength_) == ldcp_sdk::no_error)
            RCLCPP_INFO(node_->get_logger(), "Shadow filter strength set to %d", shadow_filter_strength_);
          else
            RCLCPP_WARN(node_->get_logger(), "Unable to set shadow filter strength");
        }

        if (receiver_sensitivity_boost_ != DEFAULT_RECEIVER_SENSITIVITY_BOOST) {
          if (device_->setReceiverSensitivityBoost(receiver_sensitivity_boost_) == ldcp_sdk::no_error) {
            RCLCPP_INFO(node_->get_logger(), "Receiver sensitivity boost %d applied", receiver_sensitivity_boost_);
            int current_receiver_sensitivity = 0;
            if (device_->getReceiverSensitivityValue(current_receiver_sensitivity) == ldcp_sdk::no_error)
              RCLCPP_INFO(node_->get_logger(), "Current receiver sensitivity: %d", current_receiver_sensitivity);
          }
        }

        device_->startMeasurement();
        device_->startStreaming();

        int beam_count = 0;
        switch (scan_frequency) {
          case 10: beam_count = 3072; break;
          case 15: beam_count = 2048; break;
          case 20: beam_count = 1536; break;
          case 25:
          case 30: beam_count = 1024; break;
          default: beam_count = 2048; break;
        }
        int beam_index_min = std::ceil(angle_min_ * beam_count / (2 * M_PI));
        int beam_index_max = std::floor(angle_max_ * beam_count / (2 * M_PI));
        int beam_index_excluded_min = std::ceil(angle_excluded_min_ * beam_count / (2 * M_PI));
        int beam_index_excluded_max = std::floor(angle_excluded_max_ * beam_count / (2 * M_PI));

        sensor_msgs::msg::LaserScan laser_scan;
        laser_scan.header.frame_id = frame_id_;
        laser_scan.angle_min = angle_min_;
        laser_scan.angle_max = angle_max_;
        laser_scan.angle_increment = 2 * M_PI / beam_count * average_factor_;
        laser_scan.time_increment = 1.0 / scan_frequency / beam_count * average_factor_;
        laser_scan.scan_time = 1.0 / scan_frequency;
        laser_scan.range_min = range_min_;
        laser_scan.range_max = range_max_;

        auto readScanBlock = [&](ldcp_sdk::ScanBlock& scan_block) {
          if (device_->readScanBlock(scan_block) != ldcp_sdk::no_error)
            throw std::exception();
        };

        auto updateLaserScan = [&](const ldcp_sdk::ScanBlock& scan_block) {
          int block_size = scan_block.layers[0].ranges.size();
          for (int i = 0; i < block_size; i++) {
            int beam_index = block_size * (scan_block.block_id - 4) + i;
            if (beam_index < beam_index_min || beam_index > beam_index_max)
              continue;
            if (beam_index >= beam_index_excluded_min && beam_index <= beam_index_excluded_max)
              continue;
            if (scan_block.layers[0].ranges[i] != 0) {
              laser_scan.ranges[beam_index - beam_index_min] = scan_block.layers[0].ranges[i] * 0.002;
              laser_scan.intensities[beam_index - beam_index_min] = scan_block.layers[0].intensities[i];
            }
            else {
              laser_scan.ranges[beam_index - beam_index_min] = std::numeric_limits<float>::infinity();
              laser_scan.intensities[beam_index - beam_index_min] = 0;
            }
          }
        };

        while (rclcpp::ok() && !quit_driver_.load()) {
          laser_scan.ranges.resize(beam_index_max - beam_index_min + 1);
          laser_scan.intensities.resize(beam_index_max - beam_index_min + 1);

          std::fill(laser_scan.ranges.begin(), laser_scan.ranges.end(), 0.0);
          std::fill(laser_scan.intensities.begin(), laser_scan.intensities.end(), 0.0);

          ldcp_sdk::ScanBlock scan_block;
          try {
            do {
              readScanBlock(scan_block);
            } while (scan_block.block_id != 0);

            laser_scan.header.stamp = node_->now();

            while (scan_block.block_id != 8 - 1) {
              updateLaserScan(scan_block);
              readScanBlock(scan_block);
            }
            updateLaserScan(scan_block);

            if (average_factor_ != 1) {
              int final_size = laser_scan.ranges.size() / average_factor_;
              for (int i = 0; i < final_size; i++) {
                double ranges_total = 0, intensities_total = 0;
                int count = 0;
                for (int j = 0; j < average_factor_; j++) {
                  int index = i * average_factor_ + j;
                  if (laser_scan.ranges[index] != 0) {
                    ranges_total += laser_scan.ranges[index];
                    intensities_total += laser_scan.intensities[index];
                    count++;
                  }
                }

                if (count > 0) {
                  laser_scan.ranges[i] = ranges_total / count;
                  laser_scan.intensities[i] = (int)(intensities_total / count);
                }
                else {
                  laser_scan.ranges[i] = 0;
                  laser_scan.intensities[i] = 0;
                }
              }

              laser_scan.ranges.resize(final_size);
              laser_scan.intensities.resize(final_size);
            }

            laser_scan_publisher->publish(laser_scan);

            if (hibernation_requested_.load()) {
              device_->stopMeasurement();
              RCLCPP_INFO(node_->get_logger(), "Device brought into hibernation");
              rclcpp::Rate loop_rate(10);
              while (hibernation_requested_.load())
                loop_rate.sleep();
              device_->startMeasurement();
              RCLCPP_INFO(node_->get_logger(), "Woken up from hibernation");
            }
          }
          catch (const std::exception&) {
            RCLCPP_WARN(node_->get_logger(), "Error reading data from device");
            break;
          }
        }

        device_->stopStreaming();
      }
      else
        device_->reboot();

      lock.lock();
      device_->close();

      if (!reboot_required)
        RCLCPP_INFO(node_->get_logger(), "Device closed");
      else
        RCLCPP_INFO(node_->get_logger(), "Device rebooted");
    }
    else {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "Waiting for device... [%s]", device_address_.c_str());
      loop_rate.sleep();
    }
  }
}

bool LidarDriver::querySerialService(const std::shared_ptr<agv_interfaces::srv::QuerySerial::Request> request,
                                     std::shared_ptr<agv_interfaces::srv::QuerySerial::Response> response)
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    std::string serial;
    if (device_->querySerial(serial) == ldcp_sdk::no_error) {
      response->serial = serial;
      return true;
    }
  }
  return false;
}

bool LidarDriver::queryFirmwareVersion(const std::shared_ptr<agv_interfaces::srv::QueryFirmwareVersion::Request> request,
                                       std::shared_ptr<agv_interfaces::srv::QueryFirmwareVersion::Response> response)
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    std::string firmware_version;
    if (device_->queryFirmwareVersion(firmware_version) == ldcp_sdk::no_error) {
      response->firmware_version = firmware_version;
      return true;
    }
  }
  return false;
}

bool LidarDriver::queryHardwareVersion(const std::shared_ptr<agv_interfaces::srv::QueryHardwareVersion::Request> request,
                                       std::shared_ptr<agv_interfaces::srv::QueryHardwareVersion::Response> response)
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    std::string hardware_version;
    if (device_->queryHardwareVersion(hardware_version) == ldcp_sdk::no_error) {
      response->hardware_version = hardware_version;
      return true;
    }
  }
  return false;
}

bool LidarDriver::requestHibernationService(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                            std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    hibernation_requested_ = true;
    return true;
  }
  return false;
}

bool LidarDriver::requestWakeUpService(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                       std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    hibernation_requested_ = false;
    return true;
  }
  return false;
}

bool LidarDriver::quitDriverService(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                    std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  quit_driver_ = true;
  return true;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  LidarDriver lidarDriver;
  lidarDriver.run();

  return 0;
}
