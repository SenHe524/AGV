// #include <chrono>
// #include <functional>
// #include <memory>
// #include <string>

// #include "rclcpp/rclcpp.hpp"

// #include "tf2/exceptions.h"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/buffer.h"
// #include "tf2_ros/create_timer_ros.h"
// #include "tf2_ros/message_filter.h"

// #ifdef TF2_CPP_HEADERS
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// #else
// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
// #endif

// #include "geometry_msgs/msg/transform_stamped.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "sensor_msgs/msg/laser_scan.hpp"
// #include "sensor_msgs/msg/point_cloud2.hpp"

// #include "laser_geometry/laser_geometry.hpp"


// using std::placeholders::_1;
// using namespace std::chrono_literals;


// class LaserTransformerPublisher : public rclcpp::Node
// {
// public:
//     LaserTransformerPublisher()
//     : Node("pointcloud2_pub") {
//         typedef std::chrono::duration<int> seconds_type;
//         seconds_type buffer_timeout(1);
//         this->declare_parameter("pub_name");
//         this->declare_parameter("sub_name");
//         this->declare_parameter("frame_id");
//         this->get_parameter_or("pub_name", pub_name, PUB_NAME);
//         this->get_parameter_or("sub_name", sub_name, SUB_NAME);
//         this->get_parameter_or("frame_id", frame_id, FRAME_ID);
//         // Publisher for pointcloud messages
//         pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pub_name, 10);
//         // TF2 message buffer
//         tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
//         auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
//             this->get_node_base_interface(),
//             this->get_node_timers_interface()
//         );
//         tf_buffer_->setCreateTimerInterface(timer_interface);
//         transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
//         laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(sub_name, rclcpp::SensorDataQoS(), 
//     std::bind(&LaserTransformerPublisher::on_laser_scan_callback, this, std::placeholders::_1));
//     }

//     const std::string PUB_NAME = "pointcloud2_pub";
//     const std::string SUB_NAME = "scan";
//     const std::string FRAME_ID = "laser_frame";
//   private:
//     std::string pub_name;
//     std::string sub_name;
//     std::string frame_id;
//     std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
//     std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
//     rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_{nullptr};
//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
//     laser_geometry::LaserProjection projector_;

//     void on_laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_in) {
//         sensor_msgs::msg::PointCloud2 cloud;
//         // printf("I'm here!-----------------------");
//         try {
//             projector_.transformLaserScanToPointCloud(frame_id, *scan_in, cloud, *tf_buffer_);
//         } catch (tf2::TransformException & ex) {
//             RCLCPP_WARN(this->get_logger(), "Failure %s\n", ex.what());
//         }
//         pointcloud_publisher_->publish(cloud);
//     }
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<LaserTransformerPublisher>());
//   rclcpp::shutdown();
//   return 0;
// }


#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"

#ifdef TF2_CPP_HEADERS
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#endif

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "laser_geometry/laser_geometry.hpp"
#include "message_filters/subscriber.h"


using std::placeholders::_1;
using namespace std::chrono_literals;


class LaserTransformerPublisher : public rclcpp::Node
{
public:
    LaserTransformerPublisher()
    : Node("pointcloud2_pub") {
        typedef std::chrono::duration<int> seconds_type;
        seconds_type buffer_timeout(1);
            this->declare_parameter("pub_name");
            this->declare_parameter("sub_name");
            this->declare_parameter("frame_id");
            // this->declare_parameter("tf_frame_id");
            this->get_parameter_or("pub_name", pub_name, PUB_NAME);
            this->get_parameter_or("sub_name", sub_name, SUB_NAME);
            this->get_parameter_or("frame_id", frame_id, FRAME_ID);
            // this->get_parameter_or("tf_frame_id", tf_frame_id, TF_FRAME_ID);
        // Publisher for pointcloud messages
        pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pub_name, 10);
        // TF2 message buffer
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(),
            this->get_node_timers_interface()
        );
        tf_buffer_->setCreateTimerInterface(timer_interface);
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        laser_sub_.subscribe(this, sub_name);
        tf_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
            laser_sub_, *tf_buffer_, frame_id, 10, this->get_node_logging_interface(),
            this->get_node_clock_interface(), buffer_timeout
        );
        tf_filter_->registerCallback(&LaserTransformerPublisher::on_laser_scan_callback, this);
    }
    const std::string PUB_NAME = "pointcloud2_pub";
    const std::string SUB_NAME = "scan";
    const std::string FRAME_ID = "laser_frame";
    // const std::string TF_FRAME_ID = "laser_frame";
    private:
    std::string pub_name;
    std::string sub_name;
    std::string frame_id;
    // std::string tf_frame_id;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_{nullptr};
    message_filters::Subscriber<sensor_msgs::msg::LaserScan> laser_sub_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> tf_filter_;
    laser_geometry::LaserProjection projector_;

    void on_laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_in) {
        sensor_msgs::msg::PointCloud2 cloud;
        try {
            projector_.transformLaserScanToPointCloud(frame_id, *scan_in, cloud, *tf_buffer_);
        } catch (tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Failure %s\n", ex.what());
        }
        pointcloud_publisher_->publish(cloud);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserTransformerPublisher>());
    rclcpp::shutdown();
    return 0;
}
