//  C++处理时间的库
#include <chrono>

#include "rclcpp/rclcpp.hpp"
//  1.导入消息类型文件
#include "agv_interfaces/msg/agv_velo.hpp"
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
        velo_pub = this->create_publisher<agv_interfaces::msg::AgvVelo>("velo_msgs", 10);
        timer_ = this->create_wall_timer(1000ms, std::bind(&velo_pub_test::timer_callback, this));
        this->v1 = 0;
        this->v2 = 0;
        this->v3 = 0;
        this->v4 = 0;
        this->declare_parameter<std::int64_t>("v1", this->v1);
        this->declare_parameter<std::int64_t>("v2", this->v2);
        this->declare_parameter<std::int64_t>("v3", this->v3);
        this->declare_parameter<std::int64_t>("v4", this->v4);
    }

private:
    //  3.声明发布者订阅者
    rclcpp::Publisher<agv_interfaces::msg::AgvVelo>::SharedPtr velo_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    int64_t v1, v2, v3, v4;
    void timer_callback()
    {
        //  回调函数处理
        agv_interfaces::msg::AgvVelo velo_pub_;
        this->get_parameter("v1", this->v1);
        this->get_parameter("v2", this->v2);
        this->get_parameter("v3", this->v3);
        this->get_parameter("v4", this->v4);
        velo_pub_.v1.data = this->v1;
        velo_pub_.v2.data = this->v2;
        velo_pub_.v3.data = this->v3;
        velo_pub_.v4.data = this->v4;
        velo_pub->publish(velo_pub_);
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
