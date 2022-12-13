#include "canopen_node.hpp"
using namespace std::chrono_literals;


namespace canopen_{

void canopen_node::_initSerial(void)
{
    RCLCPP_INFO(canopen_node::get_logger(), "串口节点启动中...");
    protocol_config.serial_baut_ = 115200;
    protocol_config.serial_address_ = "/dev/ttyUSB0";
    serial_pro = std::make_shared<serial_protocol::SerialProtocol>(protocol_config);
    velo_sub = this->create_subscription<agv_interfaces::msg::AgvVelo>("velo_msgs", 10, 
    std::bind(&canopen_node::velo_callback, this, std::placeholders::_1));
    odometry_pub = this->create_publisher<agv_interfaces::msg::AgvOdometry>("odometry_data", 10);
    timer_ = this->create_wall_timer(5000ms, std::bind(&canopen_node::timer_callback, this));
    this->declare_parameter<std::int64_t>("select", select);
    //  设置接收到数据后的回调函数
    serial_pro->SetDataRecvCallback([&](const std::uint8_t* data, const std::uint8_t len) -> void
    {
        int cnt = fish_protocol::inverse_frame(rx_buf, data, len, func);
        if(cnt)
        {
            std::cout << "receive:" << cnt << std::endl;
            // for(int i = 0; i < len; i++)
            // {
            //     printf("%x ", data[i]);
            // }
            // std::cout << std::endl;
            canopen_node::data_analysis(rx_buf, cnt);
        }
    });
}

void canopen_node::velo_callback(const agv_interfaces::msg::AgvVelo::SharedPtr velo_msg)
{
    // //  回调函数处理
    if( (v1 != velo_msg->v1.data) || 
        (v2 != velo_msg->v2.data) || 
        (v3 != velo_msg->v3.data) || 
        (v4 != velo_msg->v4.data)
        )
    {
        v1 = velo_msg->v1.data;
        v2 = velo_msg->v2.data;
        v3 = velo_msg->v3.data;
        v4 = velo_msg->v4.data;
        speed_cmd(velo_msg->v1.data, velo_msg->v2.data, velo_msg->v3.data, velo_msg->v4.data);
    }
}
void canopen_node::timer_callback(void)
{
    this->get_parameter("select", this->select);
    if(select == 0)
    {

    }
    else if(select == 1)
    {

    }
    else if(select == 2)
    {

    }
    else if(select == 3)
    {

    }
    else if(select == 4)
    {

    }
    else if(select == 5)
    {

    }
    else if(select == 6)
    {

    }
    else if(select == 7)
    {

    }
    else if(select == 8)
    {

    }
    else if(select == 9)
    {

    }
}
void canopen_node::data_analysis(const std::uint8_t* data, const std::uint8_t len){
    switch (canopen_::func)
    {
        case SET_ENABLE:
        case SET_DISENABLE:
        case IS_ENABLE:
        case CLEAR_FAULT:
        case IS_FAULT:
        case SET_STOP:
        case STOP_TO_ENABLE:
            control_analysis(data, len);
            break;
        case SPEED:
            break;
        case SET_PARAM:
        case GET_PARAM:
            param_analysis(data, len);
            break;
        case ODOMETRY:
            odometry_analysis(data);            
            break;
        case IMU:
            imu_analysis(data);            
            break;
        default:
            printf("unknown func:%x\n", canopen_::func);
            break;
    }
}
void canopen_node::control_analysis(const std::uint8_t* control_retdata, const std::uint8_t len)
{
    printf("control_analysis::func = %x\n", canopen_::func);
    if(len != 4)
        return ;
    control_ret.flag = 1;
    control_ret.func = canopen_::func;
    control_ret.cmd_ret[0] = control_retdata[0];
    control_ret.cmd_ret[1] = control_retdata[1];
    control_ret.cmd_ret[2] = control_retdata[2];
    control_ret.cmd_ret[3] = control_retdata[3];
}
void canopen_node::speed_analysis(const std::uint8_t* speed_retdata, const std::uint8_t len)
{
    printf("speed_analysis::func = %x\n", canopen_::func);
    if(len != 4)
        return ;
    velocity_ret.flag = 1;
    velocity_ret.func = canopen_::func;
    velocity_ret.velo_ret[0] = speed_retdata[0];
    velocity_ret.velo_ret[1] = speed_retdata[1];
    velocity_ret.velo_ret[2] = speed_retdata[2];
    velocity_ret.velo_ret[3] = speed_retdata[3];
}

void canopen_node::param_analysis(const std::uint8_t* param_retdata, const std::uint8_t len)
{
    printf("param_analysis::func = %x\n", canopen_::func);
    switch (canopen_::func)
    {
        case SET_PARAM:
            if(len != 6)
                return ;
            param_set_ret.flag = 1;
            param_set_ret.func = canopen_::func;
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
                if(len != 11)
                    return ;
                param_get16_ret.flag = 1;
                param_get16_ret.func = canopen_::func;
                param_get16_ret.reg.data8[0] = param_retdata[0];
                param_get16_ret.reg.data8[1] = param_retdata[1];
                for(int i = 0; i < 4; i++)
                {
                    param_get16_ret.get_ret[i].data8[0] = param_retdata[3+i*2];
                    param_get16_ret.get_ret[i].data8[1] = param_retdata[4+i*2];
                }
            }
            else
            {
                if(len != 18)
                    return ;
                param_get32_ret.flag = 1;
                param_get32_ret.func = canopen_::func;
                param_get32_ret.reg.data8[0] = param_retdata[0];
                param_get32_ret.reg.data8[1] = param_retdata[1];
                for(int i = 0; i < 4; i++)
                {
                    param_get32_ret.get_ret[i].data8[0] = param_retdata[3+i*4];
                    param_get32_ret.get_ret[i].data8[1] = param_retdata[4+i*4];
                    param_get32_ret.get_ret[i].data8[2] = param_retdata[5+i*4];
                    param_get32_ret.get_ret[i].data8[3] = param_retdata[6+i*4];
                }
            }
            break;
        default:
            break;
    }
}
void canopen_node::odometry_analysis(const std::uint8_t* odometry_retdata)
{
    printf("odometry_analysis::func = %x\n", canopen_::func);
    for(int i = 0; i < 4; i++)
    {
        odometry_data_.count_ret[i].data8[0] = odometry_retdata[i*4];
        odometry_data_.count_ret[i].data8[1] = odometry_retdata[i*4+1];
        odometry_data_.count_ret[i].data8[2] = odometry_retdata[i*4+2];
        odometry_data_.count_ret[i].data8[3] = odometry_retdata[i*4+3];
    }
    for(int i = 0; i < 4; i++)
    {
        odometry_data_.velo_ret[i].data8[0] = odometry_retdata[i*2+16];
        odometry_data_.velo_ret[i].data8[1] = odometry_retdata[i*2+17];
    }
    agv_interfaces::msg::AgvOdometry odo_data;
    odo_data.countbr.data = odometry_data_.count_ret[0].data_int32;
    odo_data.countbl.data = odometry_data_.count_ret[1].data_int32;
    odo_data.countfr.data = odometry_data_.count_ret[2].data_int32;
    odo_data.countfl.data = odometry_data_.count_ret[3].data_int32;
    odo_data.vbr.data = odometry_data_.velo_ret[0].data_int16;
    odo_data.vbl.data = odometry_data_.velo_ret[1].data_int16;
    odo_data.vfr.data = odometry_data_.velo_ret[2].data_int16;
    odo_data.vfl.data = odometry_data_.velo_ret[3].data_int16;
    odometry_pub->publish(odo_data);
}

void canopen_node::imu_analysis(const std::uint8_t* imu_retdata)
{
    printf("imu_analysis::func = %x\n", canopen_::func);
}
//  发送数组
void canopen_node::senddata(const unsigned char* buf, uint8_t len)
{
    serial_pro->ProtocolSenduint8_t(buf, len);
}
//  发送字符串
void canopen_node::senddata(const std::string& data)
{
    serial_pro->ProtocolSendString(data);
}


void canopen_node::control_cmd(uint8_t func)
{
    uint8_t buf[4] = {0x01, 0x01, 0x01, 0x01};
    std::cout << "control_cmd test！" << std::endl;
    int cnt = fish_protocol::frame_packing(buf, tx_buf, 4, func);
    this->senddata(tx_buf, cnt);
    std::cout << std::endl;
    // RCLCPP_INFO(this->get_logger(),"send：%d", cnt);
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
    int cnt = fish_protocol::frame_packing(buf, tx_buf, 8, 0x08);
    this->senddata(tx_buf, cnt);
    std::cout << std::endl;
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
    int cnt = fish_protocol::frame_packing(buf, tx_buf, 10, 0x09);
    this->senddata(tx_buf, cnt);
    for(int i = 0; i < cnt; i++)
    {
        printf("%x ", tx_buf[i]);
    }
    std::cout << std::endl;
}
void canopen_node::param_get_cmd(const uint16_t reg)
{
    union_uint16 reg_;
    uint8_t buf[2];
    std::cout << "param_get_cmd test！" << std::endl;
    reg_.data_uint16 = reg;
    buf[0] = reg_.data8[0];
    buf[1] = reg_.data8[1];
    int cnt = fish_protocol::frame_packing(buf, tx_buf, 2, 0x0A);
    this->senddata(tx_buf, cnt);
    for(int i = 0; i < cnt; i++)
    {
        printf("%x ", tx_buf[i]);
    }
    std::cout << std::endl;
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
        param_get16_ret.get_ret[i].data_uint16 = 0;
        param_get32_ret.get_ret[i].data_int32 = 0;
    }
}

int8_t canopen_node::setenable(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    clearcontrolbuf();
    control_cmd(0x01);
    while(!control_ret.flag || (control_ret.func != 0x01))
    {
        i++;
        rate.sleep();
        // printf("I = %d\n", i);
        if(i == 0xFFFF)
            return -1;
    }
    printf("setenable: %d\n", i);
    for(i = 0; i < 4; i++)
    {
        if(control_ret.cmd_ret[i] != 1)
            return 0;
    }
    printf("\n");
    return 1;
}
int8_t canopen_node::setdisable(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    clearcontrolbuf();
    control_cmd(0x02);
    while(!control_ret.flag || (control_ret.func != 0x02))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return -1;
    }
    printf("setdisable: %d\n", i);
    for(i = 0; i < 4; i++)
    {
        if(control_ret.cmd_ret[i] != 1)
            return 0;
    }
    printf("\n");
    return 1;
}
int8_t canopen_node::isenable(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    clearcontrolbuf();
    control_cmd(0x03);
    while(!control_ret.flag || (control_ret.func != 0x03))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return -1;
    }
    printf("isenable: %d\n", i);
    for(i = 0; i < 4; i++)
    {
        if(control_ret.cmd_ret[i] != 1)
            return 0;
        printf("0x%x ", control_ret.cmd_ret[i]);
    }
    printf("\n");
    return 1;
}
int8_t canopen_node::clearfault(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    clearcontrolbuf();
    control_cmd(0x04);
    while(!control_ret.flag || (control_ret.func != 0x04))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return -1;
    }
    printf("clearfault: %d\n", i);
    for(i = 0; i < 4; i++)
    {
        if(control_ret.cmd_ret[i] != 1)
            return 0;
        printf("0x%x ", control_ret.cmd_ret[i]);
    }
    printf("\n");
    return 1;
}
int8_t canopen_node::isfault(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    clearcontrolbuf();
    control_cmd(0x05);
    while(!control_ret.flag || (control_ret.func != 0x05))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return -1;
    }
    printf("isfault: %d\n", i);
    for(i = 0; i < 4; i++)
    {
        if(control_ret.cmd_ret[i] == 1)
            return 1;
    }
    printf("\n");
    return 0;
}
int8_t canopen_node::quickstop(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    clearcontrolbuf();
    control_cmd(0x06);
    while(!control_ret.flag || (control_ret.func != 0x06))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return -1;
    }
    printf("quickstop: %d\n", i);
    for(i = 0; i < 4; i++)
    {
        if(control_ret.cmd_ret[i]!=1)
            return 0;
    }
    printf("\n");
    return 1;
}
int8_t canopen_node::quickstop_toenable(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    clearcontrolbuf();
    control_cmd(0x07);
    while(!control_ret.flag || (control_ret.func != 0x07))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return -1;
    }
    printf("quickstop_toenable: %d\n", i);
    for(i = 0; i < 4; i++)
    {
        if(control_ret.cmd_ret[i] != 1)
            return 0;
    }
    printf("\n");
    return 1;
}
int8_t canopen_node::setspeed(const int16_t motor1_velo, const int16_t motor2_velo, 
            const int16_t motor3_velo, const int16_t motor4_velo)
{
    uint16_t i;
    rclcpp::Rate rate(2);
    clearvelobuf();
    speed_cmd(motor1_velo, motor2_velo, motor3_velo, motor4_velo);
    while((!velocity_ret.flag) || (velocity_ret.func != 0x08))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return -1; 
    }
    return 1;
}

int8_t canopen_node::set_issave_rw(uint16_t issave)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    clearsetbuf();
    param_set_cmd(SAVE_RW, issave, issave, issave, issave);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != SAVE_RW))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return -1;
    }
    printf("set_issave_rw: %d\n", i);
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::set_lock(uint16_t lock)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    clearsetbuf();
    param_set_cmd(LOCK_METHOD, lock, lock, lock, lock);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != LOCK_METHOD))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return -1;
    }
    printf("set_lock: %d\n", i);
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::set_issave_rws(uint16_t issave)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    clearsetbuf();
    param_set_cmd(SAVE_RW_S, issave, issave, issave, issave);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != SAVE_RW_S))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return -1;
    }
    printf("set_issave_rws: %d\n", i);
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::set_Vsmooth_factor(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    clearsetbuf();
    param_set_cmd(VELO_SMOOTH_FACTOR, factor1, factor2, factor3, factor4);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != VELO_SMOOTH_FACTOR))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return -1;
    }
    printf("set_Vsmooth_factor: %d\n", i);
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }
    printf("\n");
    return 1;
}


int8_t canopen_node::set_Eratio_gain(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    clearsetbuf();
    param_set_cmd(ELEC_ERATIO_GAIN, factor1, factor2, factor3, factor4);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != ELEC_ERATIO_GAIN))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return -1;
    }
    printf("set_Eratio_gain: %d\n", i);
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::set_Eintegral_gain(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    clearsetbuf();
    param_set_cmd(ELEC_INTEGRAL, factor1, factor2, factor3, factor4);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != ELEC_INTEGRAL))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return -1;
    }
    printf("set_Eratio_gain: %d\n", i);
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::set_feedforward_ratio(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    clearsetbuf();
    param_set_cmd(FEEDFORWARD_RATIO, factor1, factor2, factor3, factor4);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != FEEDFORWARD_RATIO))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return -1;
    }
    printf("set_Eratio_gain: %d\n", i);
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::set_torque_ratio(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    clearsetbuf();
    param_set_cmd(TORQUE_RATIO, factor1, factor2, factor3, factor4);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != TORQUE_RATIO))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return -1;
    }
    printf("set_Eratio_gain: %d\n", i);
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::set_VKp(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    clearsetbuf();
    param_set_cmd(VELO_KP, factor1, factor2, factor3, factor4);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != VELO_KP))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return -1;
    }
    printf("set_Eratio_gain: %d\n", i);
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::set_VKi(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    clearsetbuf();
    param_set_cmd(VELO_KI, factor1, factor2, factor3, factor4);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != VELO_KI))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return -1;
    }
    printf("set_Eratio_gain: %d\n", i);
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::set_Vfeedforward_Kf(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    clearsetbuf();
    param_set_cmd(VELO_FEEDFORWARD_KF, factor1, factor2, factor3, factor4);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != VELO_FEEDFORWARD_KF))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return -1;
    }
    printf("set_Eratio_gain: %d\n", i);
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::set_PKp(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    clearsetbuf();
    param_set_cmd(POSI_KP, factor1, factor2, factor3, factor4);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != POSI_KP))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return -1;
    }
    printf("set_Eratio_gain: %d\n", i);
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::set_Pfeedforward_Kf(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    clearsetbuf();
    param_set_cmd(POSI_FEEDFORWARD_KF, factor1, factor2, factor3, factor4);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != POSI_FEEDFORWARD_KF))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return -1;
    }
    printf("set_Eratio_gain: %d\n", i);
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::set_accelerate_time(uint32_t time1, uint32_t time2, uint32_t time3, uint32_t time4)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    clearsetbuf();
    param_set_cmd(ACC_TIME, time1, time2, time3, time4);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != ACC_TIME))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return -1;
    }
    printf("set_accelerate_time: %d\n", i);
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::set_decelerate_time(uint32_t time1, uint32_t time2, uint32_t time3, uint32_t time4)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    clearsetbuf();
    param_set_cmd(DE_TIME, time1, time2, time3, time4);

    while(!param_set_ret.flag || (control_ret.func != 0x09) || (param_set_ret.reg.data_uint16 != DE_TIME))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return -1;
    }
    printf("set_decelerate_time: %d\n", i);
    for(i = 0; i < 4; i++)
    {
        if(param_set_ret.set_ret[i] != 1)
            return 0;
    }
    printf("\n");
    return 1;
}


int8_t canopen_node::get_count(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    cleargetbuf();
    param_get_cmd(ACTUAL_COUNT);
    while(!param_get32_ret.flag || (param_get32_ret.func != 0x0A) || (param_get32_ret.reg.data_uint16 != ACTUAL_COUNT))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::get_motor_temp(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    cleargetbuf();
    param_get_cmd(MOTOR_TEMP);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != MOTOR_TEMP))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::get_motor_status(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    cleargetbuf();
    param_get_cmd(IS_MOTOR_MOVE);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != IS_MOTOR_MOVE))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::get_hall_status(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    cleargetbuf();
    param_get_cmd(MOTOR_HALL_STATUS);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != MOTOR_HALL_STATUS))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::get_errorcode(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    cleargetbuf();
    param_get_cmd(ERROR_CODE);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != ERROR_CODE))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::get_actual_velocity(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    cleargetbuf();
    param_get_cmd(ACTUAL_VELOCITY);
    while(!param_get32_ret.flag || (param_get32_ret.func != 0x0A) || (param_get32_ret.reg.data_uint16 != ACTUAL_VELOCITY))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::get_lock(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    cleargetbuf();
    param_get_cmd(LOCK_METHOD);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != LOCK_METHOD))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::get_issave_rws(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    cleargetbuf();
    param_get_cmd(SAVE_RW_S);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != SAVE_RW_S))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::get_Vsmooth_factor(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    cleargetbuf();
    param_get_cmd(VELO_SMOOTH_FACTOR);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != VELO_SMOOTH_FACTOR))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::get_Eratio_gain(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    cleargetbuf();
    param_get_cmd(ELEC_ERATIO_GAIN);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != ELEC_ERATIO_GAIN))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::get_Eintegral_gain(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    cleargetbuf();
    param_get_cmd(ELEC_INTEGRAL);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != ELEC_INTEGRAL))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::get_feedforward_ratio(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    cleargetbuf();
    param_get_cmd(FEEDFORWARD_RATIO);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != FEEDFORWARD_RATIO))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::get_torque_ratio(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    cleargetbuf();
    param_get_cmd(TORQUE_RATIO);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != TORQUE_RATIO))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::get_VKp(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    cleargetbuf();
    param_get_cmd(VELO_KP);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != VELO_KP))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::get_VKi(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    cleargetbuf();
    param_get_cmd(VELO_KI);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != VELO_KI))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::get_Vfeedforward_Kf(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    cleargetbuf();
    param_get_cmd(VELO_FEEDFORWARD_KF);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != VELO_FEEDFORWARD_KF))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::get_PKp(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    cleargetbuf();
    param_get_cmd(POSI_KP);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != POSI_KP))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::get_Pfeedforward_Kf(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    cleargetbuf();
    param_get_cmd(POSI_FEEDFORWARD_KF);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != POSI_FEEDFORWARD_KF))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::get_accelerate_time(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    cleargetbuf();
    param_get_cmd(ACC_TIME);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != ACC_TIME))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return 0;
    }
    printf("\n");
    return 1;
}

int8_t canopen_node::get_decelerate_time(void)
{
    uint16_t i = 0;
    rclcpp::Rate rate(2);
    cleargetbuf();
    param_get_cmd(DE_TIME);
    while(!param_get16_ret.flag || (param_get16_ret.func != 0x0A) || (param_get16_ret.reg.data_uint16 != DE_TIME))
    {
        i++;
        rate.sleep();
        if(i == 0xFFFF)
            return 0;
    }
    printf("\n");
    return 1;
}


}


int main(int argc, char** argv) {
  /* 初始化rclcpp  */
  rclcpp::init(argc, argv);
  /*产生一个node_01的节点*/
  auto node = std::make_shared<canopen_::canopen_node>("serial_test");
  /* 运行节点，并检测退出信号 Ctrl+C*/
  rclcpp::spin(node);
  /* 停止运行 */
  rclcpp::shutdown();
  return 0;
}



