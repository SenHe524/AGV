//  C++处理时间的库
#include <chrono>

#include "rclcpp/rclcpp.hpp"
//  1.导入消息类型文件
#include "agv_interfaces/msg/agv_velo.hpp"
#include "std_msgs/msg/u_int16.hpp"
using namespace std::chrono_literals;

//  声明占位参数
using std::placeholders::_1;




//  声明类
class velo_pub_test: public rclcpp::Node
{
public:
    //  构造函数
    velo_pub_test(std::string name):Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "大家好，我是%s！", name.c_str());

        //  3.创建发布者
        cmd_pub = this->create_publisher<std_msgs::msg::UInt16>("cmd_msgs", 10);
        timer_ = this->create_wall_timer(1000ms, std::bind(&velo_pub_test::timer_callback, this));
        this->select = 10;
        this->declare_parameter<std::int64_t>("select", this->select);
    }

private:
    //  3.声明发布者订阅者
    rclcpp::Publisher<agv_interfaces::msg::AgvVelo>::SharedPtr velo_pub;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr cmd_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    int64_t select;
    void timer_callback()
    {
        std_msgs::msg::UInt16 se;
        int64_t se_te;
        this->get_parameter("select", se_te);
        se.data = (uint16_t)se_te;
        cmd_pub->publish(se);
    }
};



int main(int argc, char ** argv)
{
	//  初始化客户端库
    rclcpp::init(argc, argv);
    //  实例化继承了Node的类
    auto node = std::make_shared<velo_pub_test>("velo_pub_test");
    //  spin循环节点
    rclcpp::spin(node);
    //  关闭客户端库
    rclcpp::shutdown();
    return 0;
}
