#include "canopen_node.hpp"
using namespace std::chrono_literals;


namespace canopen_{
using std::placeholders::_2;
void canopen_node::_initSerial(void)
{
    serial_object = std::make_shared<serial::Serial>();
    cmd_sub = this->create_subscription<std_msgs::msg::UInt16>("cmd_msgs", 10, 
    std::bind(&canopen_node::cmd_callback, this, std::placeholders::_1));
    odometry_pub = this->create_publisher<agv_interfaces::msg::AgvOdometry>("odometry_data", 10);
    odo_pub = this->create_publisher<nav_msgs::msg::Odometry>("ego_state", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu_msg", 10);
    ret_pub = this->create_publisher<agv_interfaces::msg::Ret>("ret_msg", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&canopen_node::timer_callback, this));
    this->declare_parameter<std::int64_t>("select", select);
    this->declare_parameter<std::int64_t>("v1", this->v1);
    this->declare_parameter<std::int64_t>("v2", this->v2);
    this->declare_parameter<std::int64_t>("v3", this->v3);
    this->declare_parameter<std::int64_t>("v4", this->v4);
    ret_data.m1_16.data = 0;
    ret_data.m2_16.data = 0;
    ret_data.m3_16.data = 0;
    ret_data.m4_16.data = 0;
    ret_data.m1_32.data = 0;
    ret_data.m2_32.data = 0;
    ret_data.m3_32.data = 0;
    ret_data.m4_32.data = 0;
    // serial库串口打开
    try
    {
        RCLCPP_INFO(canopen_node::get_logger(), "串口节点启动中...");
        serial_object->setPort("/dev/ttyUSB0");//选择要开启的串口号
        serial_object->setBaudrate(115200);//设置波特率
        serial::Timeout timeOut = serial::Timeout::simpleTimeout(serial::Timeout::max());  //超时等待
        serial_object->setTimeout(timeOut);                                     
        serial_object->open(); //开启串口
    }
    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "can not open serial port,Please check the serial port cable! "); //如果开启串口失败，打印错误信息
    }

    // 如果串口打开，则驱动读取数据的线程
    if (serial_object->isOpen())
    {
        RCLCPP_INFO(this->get_logger(), "serial port opened"); //串口开启成功提示

        // 启动一个新线程读取并处理串口数据
        read_data_thread_ = std::shared_ptr<std::thread>(
            new std::thread(std::bind(&canopen_node::readRawData, this)));
    }
    else{
        RCLCPP_INFO(this->get_logger(), "serial port open failed"); //串口开启失败提示
    }
}


void canopen_node::cmd_callback(const std_msgs::msg::UInt16::SharedPtr cmd_msg)
{
    if(cmd_msg->data == 0)
    {
        set_accelerate_time(100, 100, 100, 100);
    }
    else if(cmd_msg->data  == 1)
    {
        get_accelerate_time();
    }
    else if(cmd_msg->data  == 2)
    {
        isenable();
    }
    else if(cmd_msg->data  == 3)
    {
        quickstop();
    }
    else if(cmd_msg->data  ==4)
    {
        quickstop_toenable();
    }
    else if(cmd_msg->data  ==5)
    {

    }
    else if(cmd_msg->data  ==6)
    {

    }
    // std::cout << "I'm in timercallback " << cmd_msg->data  << std::endl;
}

void canopen_node::timer_callback(void)
{
    int16_t t1,t2,t3,t4;
    this->get_parameter("v1", t1);
    this->get_parameter("v2", t2);
    this->get_parameter("v3", t3);
    this->get_parameter("v4", t4);
    if( (v1 != t1) || 
        (v2 != t2) || 
        (v3 != t3) || 
        (v4 != t4)
        )
    {
        v1 = t1;
        v2 = t2;
        v3 = t3;
        v4 = t4;
        speed_cmd(v1, v2, v3, v4);
    }
}

void canopen_node::clear_usart1cmd(void)
{

	for (uint8_t i = 0; i < data_len; i++)
		rx_buf[i] = 0;
	data_rxflag = 0;
	data_index = 0;
}
void canopen_node::data_rcv(uint8_t rxdata)
{
	switch (data_rxflag) {
		case 0: // 帧头
		{   
			if (rxdata == FIRST_CODE) {
				rx_buf[data_index++] = FIRST_CODE;
				data_rxflag = 1;
                // std::cout << "帧头:" << rxdata << std::endl;
			} else {
				data_rxflag = 0;
				data_index = 0;
				rx_buf[0] = 0x0;
			}
			break;
		}
		case 1:// 标识位
		{
			if ((rxdata >= 0x01) && (rxdata <= 0x0C)) {
				rx_buf[data_index++] = rxdata;
				data_rxflag = 2;
                // std::cout << "标识:" << rxdata << std::endl;
			} else {
				data_rxflag = 0;
				data_index = 0;
				rx_buf[0] = 0;
				rx_buf[1] = 0;
			}
			break;
		}
		case 2:// 数据位长度
		{
			// New_CMD_length为数据帧总字节数 = 帧头+标识位+长度+校验位+帧尾(5 bytes)+数据位
			data_len = rxdata + 5;
			if (data_len < DATA_BUF_MAX_LEN) {
				rx_buf[data_index++] = rxdata;
				data_rxflag = 3;
                // std::cout << "数据长度:" << rxdata << std::endl;
			} else {
				data_rxflag = 0;
				data_index = 0;
				rx_buf[0] = 0;
				rx_buf[1] = 0;
				data_len = 0;
			}
			break;
		}
		case 3:// 读取完剩余的所有字段
		{
			rx_buf[data_index++] = rxdata;
            // std::cout << "数据:" << rxdata << std::endl;
			if(data_index >= data_len && rx_buf[data_len-1] == END_CODE) {
				data_rxflag = 0;
				data_index = 0;
                // std::cout << "一个完整的帧!" << std::endl;
                data_analysis(rx_buf, data_len);
                clear_usart1cmd();
			} else if(data_index >= data_len 
				&& rx_buf[data_len-1] != END_CODE){
				clear_usart1cmd();
			}
			break;
		}
		default:
			clear_usart1cmd();
			break;
	}
}
void canopen_node::readRawData(void)
{
    uint8_t rx_data = 0;
    // DataFrame frame;

    while (rclcpp::ok()) 
    {
        // 读取一个字节数据，寻找帧头
        auto len = serial_object->read(&rx_data, 1);
        if (len < 1)
            continue;
        data_rcv(rx_data);
    }
}
void canopen_node::data_analysis(const std::uint8_t* data, const std::uint8_t len){
    int cnt = frame_pack::inverse_frame(frame_buf, data, len, func);
    if(!cnt)
    {
        return ;
    }
    if(func != 0x0B)
    {
        printf("receive: %d, func: %d\n", len, func);
        for(int i = 0; i < len; i++)
        {
            printf("%x ", data[i]);
        }
        std::cout << std::endl;
    }
    switch (canopen_node::func)
    {
        case SET_ENABLE:
        case SET_DISENABLE:
        case IS_ENABLE:
        case CLEAR_FAULT:
        case IS_FAULT:
        case SET_STOP:
        case STOP_TO_ENABLE:
            {
                control_analysis(frame_buf, cnt);
            }
            break;
        case SPEED:
            {
                speed_analysis(frame_buf, cnt);
            }
            break;
        case SET_PARAM:
        case GET_PARAM:
            {
                param_analysis(frame_buf);
            }
            break;
        case ODOMETRY_IMU:
            {
                odometry_imu_analysis(frame_buf); 
            }
            break;
        default:
            printf("unknown func:%x\n", canopen_node::func);
            break;
    }
    canopen_node::func = 0x00;
}
void canopen_node::control_analysis(const std::uint8_t* control_retdata, const std::uint8_t len)
{
    if(len != 4)
        return ;
    control_ret.flag = 1;
    control_ret.func = canopen_node::func;
    control_ret.cmd_ret[0] = control_retdata[0];
    control_ret.cmd_ret[1] = control_retdata[1];
    control_ret.cmd_ret[2] = control_retdata[2];
    control_ret.cmd_ret[3] = control_retdata[3];
}
void canopen_node::speed_analysis(const std::uint8_t* speed_retdata, const std::uint8_t len)
{
    if(len != 4)
        return ;
    velocity_ret.flag = 1;
    velocity_ret.func = canopen_node::func;
    velocity_ret.velo_ret[0] = speed_retdata[0];
    velocity_ret.velo_ret[1] = speed_retdata[1];
    velocity_ret.velo_ret[2] = speed_retdata[2];
    velocity_ret.velo_ret[3] = speed_retdata[3];
}
void canopen_node::param_analysis(const std::uint8_t* param_retdata)
{
    switch (canopen_node::func)
    {
        case SET_PARAM:
            param_set_ret.flag = 1;
            param_set_ret.func = canopen_node::func;
            param_set_ret.reg.data8[0] = param_retdata[0];
            param_set_ret.reg.data8[1] = param_retdata[1];
            param_set_ret.set_ret[0] = param_retdata[2];
            param_set_ret.set_ret[1] = param_retdata[3];
            param_set_ret.set_ret[2] = param_retdata[4];
            param_set_ret.set_ret[3] = param_retdata[5];
            break;
        case GET_PARAM:
            if(param_retdata[2])
            {
                param_get16_ret.flag = 1;
                param_get16_ret.func = canopen_node::func;
                param_get16_ret.reg.data8[0] = param_retdata[0];
                param_get16_ret.reg.data8[1] = param_retdata[1];
                for(int i = 0; i < 4; i++)
                {
                    param_get16_ret.get_ret[i] = *(uint16_t*)(param_retdata+i*2+3);
                }
                ret_data.m1_16.data = param_get16_ret.get_ret[0];
                ret_data.m2_16.data = param_get16_ret.get_ret[1];
                ret_data.m3_16.data = param_get16_ret.get_ret[2];
                ret_data.m4_16.data = param_get16_ret.get_ret[3];
                ret_data.m1_32.data = 0;
                ret_data.m2_32.data = 0;
                ret_data.m3_32.data = 0;
                ret_data.m4_32.data = 0;
                ret_pub->publish(ret_data);
            }
            else
            {
                param_get32_ret.flag = 1;
                param_get32_ret.func = canopen_node::func;
                param_get32_ret.reg.data8[0] = param_retdata[0];
                param_get32_ret.reg.data8[1] = param_retdata[1];
                for(int i = 0; i < 4; i++)
                {
                    param_get32_ret.get_ret[i] = *(int32_t*)(param_retdata+i*4+3);
                }
                ret_data.m1_16.data = 0;
                ret_data.m2_16.data = 0;
                ret_data.m3_16.data = 0;
                ret_data.m4_16.data = 0;
                ret_data.m1_32.data = param_get32_ret.get_ret[0];
                ret_data.m2_32.data = param_get32_ret.get_ret[1];
                ret_data.m3_32.data = param_get32_ret.get_ret[2];
                ret_data.m4_32.data = param_get32_ret.get_ret[3];
                ret_pub->publish(ret_data);
            }
            break;
        default:
            break;
    }
}
void canopen_node::odometry_imu_analysis(const std::uint8_t* odometry_imu_retdata)
{
    float vl = 0.0, vr = 0.0;
    float vx = 0.0, vth = 0.0;
    float delta_th = 0.0, delta_x = 0.0, delta_y = 0.0;
    for(int i = 0; i < 4; i++)
    {
        odometry_data_.count_ret[i] = *(int32_t*)(odometry_imu_retdata+i*4);
    }
    for(int i = 0; i < 4; i++)
    {
        odometry_data_.velo_ret[i] = *(int16_t*)(odometry_imu_retdata+i*2+16);
    }
    memcpy(&imu_info, odometry_imu_retdata+24, 52);
    agv_interfaces::msg::AgvOdometry odo_data;
    sensor_msgs::msg::Imu imu_msg;

    odo_data.countbr.data = odometry_data_.count_ret[0];
    odo_data.countbl.data = odometry_data_.count_ret[1];
    odo_data.countfr.data = odometry_data_.count_ret[2];
    odo_data.countfl.data = odometry_data_.count_ret[3];
    odo_data.vbr.data = odometry_data_.velo_ret[0];
    odo_data.vbl.data = odometry_data_.velo_ret[1];
    odo_data.vfr.data = odometry_data_.velo_ret[2];
    odo_data.vfl.data = odometry_data_.velo_ret[3];

    // 计算两个周期之间的时间差
    static rclcpp::Time last_time_ = this->now();
    current_time_ = this->now();
    float dt = (current_time_.seconds() - last_time_.seconds());
    last_time_ = current_time_;

    vl = ((float)odo_data.vfl.data/600.0f) * 0.2f * M_PI;
    vr = ((float)odo_data.vfr.data/600.0f) * 0.2f * M_PI;
    vx = (vl+vr)/2.0f;
    vth = (vr-vl)/0.5262f;
    // RCLCPP_INFO(this->get_logger(), "dt=%f left_speed=%f right_speed=%f vx=%f vth=%f", dt, vl, vr, vx, vth);
    // 计算里程计单周期内的姿态
    delta_x = vx * cos(odom_th_) * dt;
    delta_y = vx * sin(odom_th_) * dt;
    delta_th = vth * dt;
    // 计算里程计的累积姿态
    odom_x_  += delta_x;
    odom_y_  += delta_y;
    odom_th_ += delta_th;
    // 校正姿态角度，让机器人处于-180~180度之间
    if(odom_th_ > M_PI) 
        odom_th_ -= M_PI*2;
    else if(odom_th_ < (-M_PI)) 
        odom_th_ += M_PI*2;

    //std_msgs/Header header
    imu_msg.header.frame_id = "imu_link";
    imu_msg.header.stamp = this->get_clock()->now();
    // geometry_msgs/Vector3 linear_acceleration
    imu_msg.linear_acceleration.x = imu_info.accel_x;
    imu_msg.linear_acceleration.y = imu_info.accel_y;
    imu_msg.linear_acceleration.z = imu_info.accel_z;
    // geometry_msgs/Vector3 angular_velocity
    imu_info.angle_x *= M_PI / 180.0f;
    imu_info.angle_y *= M_PI / 180.0f;
    imu_info.angle_z *= M_PI / 180.0f;
    imu_msg.angular_velocity.x = imu_info.angle_x;// * M_PI / 180.0f
    imu_msg.angular_velocity.y = imu_info.angle_y;
    imu_msg.angular_velocity.z = imu_info.angle_z;
    
    
    // geometry_msgs/Quaternion orientation
    // tf2::Quaternion q;
    // q.setRPY(imu_info.roll, imu_info.pitch, imu_info.yaw);
    // imu_msg.orientation.x = q[0];
    // imu_msg.orientation.y = q[1];
    // imu_msg.orientation.z = q[2];
    // imu_msg.orientation.w = q[3];
    imu_msg.orientation.x = imu_info.quaternion_data0;
    imu_msg.orientation.y = imu_info.quaternion_data1;
    imu_msg.orientation.z = imu_info.quaternion_data2;
    imu_msg.orientation.w = imu_info.quaternion_data3;

    // float64[9] linear_acceleration_covariance # Row major x, y z
    imu_msg.linear_acceleration_covariance = {  0.04, 0.00, 0.00,
                                                0.00, 0.04, 0.00,
                                                0.00, 0.00, 0.04};
    // float64[9] angular_velocity_covariance # Row major about x, y, z axes
    imu_msg.angular_velocity_covariance = { 0.02, 0.00, 0.00,
                                            0.00, 0.02, 0.00,
                                            0.00, 0.00, 0.02};
    // float64[9] orientation_covariance # Row major about x, y, z axes
    imu_msg.orientation_covariance = {  0.0025, 0.0000, 0.0000,
                                        0.0000, 0.0025, 0.0000,
                                        0.0000, 0.0000, 0.0025};

    odom_pub(vx,vth);
    odometry_pub->publish(odo_data);
    imu_pub->publish(imu_msg);
}

void canopen_node::odom_pub(float vx, float vth)
{
    auto odom_msg = nav_msgs::msg::Odometry();
    //里程数据计算
    odom_msg.header.frame_id = "odom";
    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.pose.pose.position.x = odom_x_;
    odom_msg.pose.pose.position.y = odom_y_;
    odom_msg.pose.pose.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, odom_th_);
    odom_msg.child_frame_id = "odom_link";
    odom_msg.pose.pose.orientation.x = q[0];
    odom_msg.pose.pose.orientation.y = q[1];
    odom_msg.pose.pose.orientation.z = q[2];
    odom_msg.pose.pose.orientation.w = q[3];

    const double odom_pose_covariance[36] = {1e-3, 0, 0, 0, 0, 0,

                                             0, 1e-3, 0, 0, 0, 0,

                                             0, 0, 1e6, 0, 0, 0,
                                             0, 0, 0, 1e6, 0, 0,

                                             0, 0, 0, 0, 1e6, 0,

                                             0, 0, 0, 0, 0, 1e3};
    const double odom_pose_covariance2[36]= {1e-9,    0,    0,   0,   0,    0,
										      0, 1e-3, 1e-9,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9 };

    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = 0.00;
    odom_msg.twist.twist.linear.z = 0.00;

    odom_msg.twist.twist.angular.x = 0.00;
    odom_msg.twist.twist.angular.y = 0.00;
    odom_msg.twist.twist.angular.z = vth;

    const double odom_twist_covariance[36] = { 1e-3, 0, 0, 0, 0, 0,
                                                  0, 1e-3, 0, 0, 0, 0,

                                                  0, 0, 1e6, 0, 0, 0,

                                                  0, 0, 0, 1e6, 0, 0,

                                                  0, 0, 0, 0, 1e6, 0,

                                                  0, 0, 0, 0, 0, 1e3};
    const double odom_twist_covariance2[36] = {1e-9,    0,    0,   0,   0,    0, 
                                                  0, 1e-3, 1e-9,   0,   0,    0,
                                                  0,    0,  1e6,   0,   0,    0,
                                                  0,    0,    0, 1e6,   0,    0,
                                                  0,    0,    0,   0, 1e6,    0,
                                                  0,    0,    0,   0,   0, 1e-9};


    if (vx == 0 && vth == 0)
        memcpy(&odom_msg.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2)),
            memcpy(&odom_msg.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
    else
        memcpy(&odom_msg.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance)),
            memcpy(&odom_msg.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));

    // 发布里程计话题
    odo_pub->publish(odom_msg);

    geometry_msgs::msg::TransformStamped trans;

    trans.header.stamp = this->get_clock()->now();
    trans.header.frame_id = "odom";
    trans.child_frame_id  = "odom_link";

    trans.transform.translation.x = odom_x_;
    trans.transform.translation.y = odom_y_;
    trans.transform.translation.z = 0.0;

    trans.transform.rotation.x = q[0];
    trans.transform.rotation.y = q[1];
    trans.transform.rotation.z = q[2];
    trans.transform.rotation.w = q[3];
    // 广播里程计TF
    tf_broadcaster_->sendTransform(trans);
}

/*********************************包装serial库发送函数***************************************************/
size_t canopen_node::senddata(const unsigned char* buf, size_t len)
{
    if(len<=0)
    return 0;
    size_t ret = 0;
    try
    {
        ret = serial_object->write(buf, len);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    return ret;
}
//  发送字符串
size_t canopen_node::senddata(const std::string& data)
{
    size_t ret = 0;
    try
    {
            ret = serial_object->write(data);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    return ret;
}
/*********************************包装serial库发送函数***************************************************/

void canopen_node::control_cmd(uint8_t func)
{
    uint8_t buf[4] = {0x01, 0x01, 0x01, 0x01};
    std::cout << "control_cmd test！" << std::endl;
    size_t cnt = frame_pack::frame_packing(buf, tx_buf, 4, func);
    if(!this->senddata(tx_buf, cnt))
    {
        std::cout << "send error!" << std::endl;
    }
}
void canopen_node::speed_cmd(const int16_t motor1_velo, const int16_t motor2_velo, 
    const int16_t motor3_velo, const int16_t motor4_velo)
{
    union_int16 velo_[4];
    uint8_t buf[8];
    std::cout << "speed_cmd test！" << std::endl;
    velo_[0].data_int16 = motor1_velo;
    velo_[1].data_int16 = motor2_velo;
    velo_[2].data_int16 = motor3_velo;
    velo_[3].data_int16 = motor4_velo;
    for(int i = 0; i < 4; i++)
    {
        buf[i*2] = velo_[i].data8[0];
        buf[i*2+1] = velo_[i].data8[1];
    }
    size_t cnt = frame_pack::frame_packing(buf, tx_buf, 8, 0x08);
    if(!this->senddata(tx_buf, cnt))
    {
        std::cout << "send error!" << std::endl;
    }
}
void canopen_node::param_set_cmd(const uint16_t reg, const uint16_t motor1_data, 
    const uint16_t motor2_data, const uint16_t motor3_data, uint16_t motor4_data)
{
    union_uint16 data[4];
    union_uint16 reg_;
    uint8_t buf[10];
    std::cout << "param_set_cmd test！" << std::endl;
    reg_.data_uint16 = reg;
    buf[0] = reg_.data8[0];
    buf[1] = reg_.data8[1];
    data[0].data_uint16 = motor1_data;
    data[1].data_uint16 = motor2_data;
    data[2].data_uint16 = motor3_data;
    data[3].data_uint16 = motor4_data;
    for(int i = 0; i < 4; i++)
    {
        buf[i*2+2] = data[i].data8[0];
        buf[i*2+3] = data[i].data8[1];
    }
    size_t cnt = frame_pack::frame_packing(buf, tx_buf, 10, 0x09);
    if(!this->senddata(tx_buf, cnt))
    {
        std::cout << "send error!" << std::endl;
    }
}
void canopen_node::param_get_cmd(const uint16_t reg)
{
    union_uint16 reg_;
    uint8_t buf[2];
    std::cout << "param_get_cmd test！" << std::endl;
    reg_.data_uint16 = reg;
    buf[0] = reg_.data8[0];
    buf[1] = reg_.data8[1];
    size_t cnt = frame_pack::frame_packing(buf, tx_buf, 2, 0x0A);
    if(!this->senddata(tx_buf, cnt))
    {
        std::cout << "send error!" << std::endl;
    }
}


void canopen_node::clearcontrolbuf(void)
{
    control_ret.flag = 0;
    control_ret.func = 0;
    for(int i = 0; i < 4; i++)
    {
        control_ret.cmd_ret[i] = 0;
    }
}
void canopen_node::clearvelobuf(void)
{
    velocity_ret.flag = 0;
    velocity_ret.func = 0;
    for(int i = 0; i < 4; i++)
    {
        velocity_ret.velo_ret[i] = 0;
    }
}
void canopen_node::clearsetbuf(void)
{
    param_set_ret.flag = 0;
    param_set_ret.func = 0;
    param_set_ret.reg.data_uint16 = 0;
    for(int i = 0; i < 4; i++)
    {
        control_ret.cmd_ret[i] = 0;
    }
}
void canopen_node::cleargetbuf(void)
{
    param_get16_ret.flag = 0;
    param_get16_ret.func = 0;
    param_get16_ret.reg.data_uint16 = 0;
    param_get32_ret.flag = 0;
    param_get32_ret.func = 0;
    param_get32_ret.reg.data_uint16 = 0;
    for(int i = 0; i < 4; i++)
    {
        param_get16_ret.get_ret[i] = 0;
        param_get32_ret.get_ret[i] = 0;
    }
}

int8_t canopen_node::setenable(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    clearcontrolbuf();
    control_cmd(0x01);
    while(!control_ret.flag || (control_ret.func != 0x01))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return -1;
    }
    for(i = 0; i < 4; i++)
    {
        if(control_ret.cmd_ret[i] != 1)
            return 0;
    }

    return 1;
}
int8_t canopen_node::setdisable(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    clearcontrolbuf();
    control_cmd(0x02);
    while(!control_ret.flag || (control_ret.func != 0x02))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return -1;
    }
    for(i = 0; i < 4; i++)
    {
        if(control_ret.cmd_ret[i] != 1)
            return 0;
    }

    return 1;
}
int8_t canopen_node::isenable(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    clearcontrolbuf();
    control_cmd(0x03);
    while(!control_ret.flag || (control_ret.func != 0x03))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return -1;
    }
    for(i = 0; i < 4; i++)
    {
        if(control_ret.cmd_ret[i] != 1)
            return 0;
    }

    return 1;
}
int8_t canopen_node::clearfault(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    clearcontrolbuf();
    control_cmd(0x04);
    while(!control_ret.flag || (control_ret.func != 0x04))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return -1;
    }
    for(i = 0; i < 4; i++)
    {
        if(control_ret.cmd_ret[i] != 1)
            return 0;
    }

    return 1;
}
int8_t canopen_node::isfault(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    clearcontrolbuf();
    control_cmd(0x05);
    while(!control_ret.flag || (control_ret.func != 0x05))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return -1;
    }
    for(i = 0; i < 4; i++)
    {
        if(control_ret.cmd_ret[i] == 1)
            return 1;
    }

    return 0;
}
int8_t canopen_node::quickstop(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    clearcontrolbuf();
    control_cmd(0x06);
    while(!control_ret.flag || (control_ret.func != 0x06))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return -1;
    }
    for(i = 0; i < 4; i++)
    {
        if(control_ret.cmd_ret[i]!=1)
            return 0;
    }

    return 1;
}
int8_t canopen_node::quickstop_toenable(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    clearcontrolbuf();
    control_cmd(0x07);
    while(!control_ret.flag || (control_ret.func != 0x07))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return -1;
    }
    for(i = 0; i < 4; i++)
    {
        if(control_ret.cmd_ret[i] != 1)
            return 0;
    }

    return 1;
}
int8_t canopen_node::setspeed(const int16_t motor1_velo, const int16_t motor2_velo, 
            const int16_t motor3_velo, const int16_t motor4_velo)
{
    uint16_t i;
    rclcpp::Rate rate(100);
    clearvelobuf();
    speed_cmd(motor1_velo, motor2_velo, motor3_velo, motor4_velo);
    while((!velocity_ret.flag) || (velocity_ret.func != 0x08))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return -1; 
    }
    return 1;
}

int8_t canopen_node::set_issave_rw(uint16_t issave)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    clearsetbuf();
    param_set_cmd(SAVE_RW, issave, issave, issave, issave);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != SAVE_RW))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return -1;
    }
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }

    return 1;
}

int8_t canopen_node::set_lock(uint16_t lock)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    clearsetbuf();
    param_set_cmd(LOCK_METHOD, lock, lock, lock, lock);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != LOCK_METHOD))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return -1;
    }
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }

    return 1;
}

int8_t canopen_node::set_issave_rws(uint16_t issave)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    clearsetbuf();
    param_set_cmd(SAVE_RW_S, issave, issave, issave, issave);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != SAVE_RW_S))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return -1;
    }
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }

    return 1;
}

int8_t canopen_node::set_Vsmooth_factor(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    clearsetbuf();
    param_set_cmd(VELO_SMOOTH_FACTOR, factor1, factor2, factor3, factor4);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != VELO_SMOOTH_FACTOR))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return -1;
    }
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }

    return 1;
}


int8_t canopen_node::set_Eratio_gain(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    clearsetbuf();
    param_set_cmd(ELEC_ERATIO_GAIN, factor1, factor2, factor3, factor4);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != ELEC_ERATIO_GAIN))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return -1;
    }
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }

    return 1;
}

int8_t canopen_node::set_Eintegral_gain(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    clearsetbuf();
    param_set_cmd(ELEC_INTEGRAL, factor1, factor2, factor3, factor4);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != ELEC_INTEGRAL))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return -1;
    }
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }

    return 1;
}

int8_t canopen_node::set_feedforward_ratio(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    clearsetbuf();
    param_set_cmd(FEEDFORWARD_RATIO, factor1, factor2, factor3, factor4);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != FEEDFORWARD_RATIO))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return -1;
    }
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }

    return 1;
}

int8_t canopen_node::set_torque_ratio(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    clearsetbuf();
    param_set_cmd(TORQUE_RATIO, factor1, factor2, factor3, factor4);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != TORQUE_RATIO))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return -1;
    }
    
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }

    return 1;
}

int8_t canopen_node::set_VKp(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    clearsetbuf();
    param_set_cmd(VELO_KP, factor1, factor2, factor3, factor4);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != VELO_KP))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return -1;
    }
    
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }

    return 1;
}

int8_t canopen_node::set_VKi(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    clearsetbuf();
    param_set_cmd(VELO_KI, factor1, factor2, factor3, factor4);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != VELO_KI))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return -1;
    }
    
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }

    return 1;
}

int8_t canopen_node::set_Vfeedforward_Kf(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    clearsetbuf();
    param_set_cmd(VELO_FEEDFORWARD_KF, factor1, factor2, factor3, factor4);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != VELO_FEEDFORWARD_KF))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return -1;
    }
    
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }

    return 1;
}

int8_t canopen_node::set_PKp(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    clearsetbuf();
    param_set_cmd(POSI_KP, factor1, factor2, factor3, factor4);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != POSI_KP))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return -1;
    }
    
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }

    return 1;
}

int8_t canopen_node::set_Pfeedforward_Kf(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    clearsetbuf();
    param_set_cmd(POSI_FEEDFORWARD_KF, factor1, factor2, factor3, factor4);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != POSI_FEEDFORWARD_KF))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return -1;
    }
    
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }

    return 1;
}

int8_t canopen_node::set_accelerate_time(uint32_t time1, uint32_t time2, uint32_t time3, uint32_t time4)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    clearsetbuf();
    param_set_cmd(ACC_TIME, time1, time2, time3, time4);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != ACC_TIME))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return -1;
    }
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }

    return 1;
}

int8_t canopen_node::set_decelerate_time(uint32_t time1, uint32_t time2, uint32_t time3, uint32_t time4)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    clearsetbuf();
    param_set_cmd(DE_TIME, time1, time2, time3, time4);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != DE_TIME))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return -1;
    }
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }

    return 1;
}


int8_t canopen_node::get_count(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    cleargetbuf();
    param_get_cmd(ACTUAL_COUNT);
    while(!param_get32_ret.flag || (param_get32_ret.func != 0x0A) || (param_get32_ret.reg.data_uint16 != ACTUAL_COUNT))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return 0;
    }

    return 1;
}

int8_t canopen_node::get_motor_temp(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    cleargetbuf();
    param_get_cmd(MOTOR_TEMP);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != MOTOR_TEMP))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return 0;
    }

    return 1;
}

int8_t canopen_node::get_motor_status(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    cleargetbuf();
    param_get_cmd(IS_MOTOR_MOVE);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != IS_MOTOR_MOVE))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return 0;
    }

    return 1;
}

int8_t canopen_node::get_hall_status(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    cleargetbuf();
    param_get_cmd(MOTOR_HALL_STATUS);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != MOTOR_HALL_STATUS))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return 0;
    }

    return 1;
}

int8_t canopen_node::get_errorcode(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    cleargetbuf();
    param_get_cmd(ERROR_CODE);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != ERROR_CODE))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return 0;
    }

    return 1;
}

int8_t canopen_node::get_actual_velocity(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    cleargetbuf();
    param_get_cmd(ACTUAL_VELOCITY);
    while(!param_get32_ret.flag || (param_get32_ret.func != 0x0A) || (param_get32_ret.reg.data_uint16 != ACTUAL_VELOCITY))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return 0;
    }

    return 1;
}

int8_t canopen_node::get_lock(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    cleargetbuf();
    param_get_cmd(LOCK_METHOD);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != LOCK_METHOD))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return 0;
    }

    return 1;
}

int8_t canopen_node::get_issave_rws(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    cleargetbuf();
    param_get_cmd(SAVE_RW_S);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != SAVE_RW_S))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return 0;
    }

    return 1;
}

int8_t canopen_node::get_Vsmooth_factor(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    cleargetbuf();
    param_get_cmd(VELO_SMOOTH_FACTOR);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != VELO_SMOOTH_FACTOR))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return 0;
    }

    return 1;
}

int8_t canopen_node::get_Eratio_gain(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    cleargetbuf();
    param_get_cmd(ELEC_ERATIO_GAIN);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != ELEC_ERATIO_GAIN))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return 0;
    }

    return 1;
}

int8_t canopen_node::get_Eintegral_gain(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    cleargetbuf();
    param_get_cmd(ELEC_INTEGRAL);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != ELEC_INTEGRAL))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return 0;
    }

    return 1;
}

int8_t canopen_node::get_feedforward_ratio(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    cleargetbuf();
    param_get_cmd(FEEDFORWARD_RATIO);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != FEEDFORWARD_RATIO))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return 0;
    }

    return 1;
}

int8_t canopen_node::get_torque_ratio(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    cleargetbuf();
    param_get_cmd(TORQUE_RATIO);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != TORQUE_RATIO))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return 0;
    }

    return 1;
}

int8_t canopen_node::get_VKp(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    cleargetbuf();
    param_get_cmd(VELO_KP);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != VELO_KP))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return 0;
    }

    return 1;
}

int8_t canopen_node::get_VKi(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    cleargetbuf();
    param_get_cmd(VELO_KI);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != VELO_KI))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return 0;
    }

    return 1;
}

int8_t canopen_node::get_Vfeedforward_Kf(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    cleargetbuf();
    param_get_cmd(VELO_FEEDFORWARD_KF);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != VELO_FEEDFORWARD_KF))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return 0;
    }

    return 1;
}

int8_t canopen_node::get_PKp(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    cleargetbuf();
    param_get_cmd(POSI_KP);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != POSI_KP))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return 0;
    }

    return 1;
}

int8_t canopen_node::get_Pfeedforward_Kf(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    cleargetbuf();
    param_get_cmd(POSI_FEEDFORWARD_KF);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != POSI_FEEDFORWARD_KF))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return 0;
    }

    return 1;
}

int8_t canopen_node::get_accelerate_time(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    cleargetbuf();
    param_get_cmd(ACC_TIME);
    while(!param_get32_ret.flag || (param_get32_ret.func != 0x0A) || (param_get32_ret.reg.data_uint16 != ACC_TIME))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return 0;
    }

    return 1;
}

int8_t canopen_node::get_decelerate_time(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(100);
    cleargetbuf();
    param_get_cmd(DE_TIME);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != DE_TIME))
    {
        i++;
        rate.sleep();
        if(i == 0xFF)
            return 0;
    }
    return 1;
}


}


int main(int argc, char** argv) {
  /* 初始化rclcpp  */
  rclcpp::init(argc, argv);
  /*产生一个node_01的节点*/
  auto node = std::make_shared<canopen_::canopen_node>("canopen_node");
  /* 运行节点，并检测退出信号 Ctrl+C*/
  rclcpp::spin(node);
  /* 停止运行 */
  rclcpp::shutdown();
  return 0;
}



