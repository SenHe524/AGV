#pragma once
#include <cstdio>
#include <stdlib.h>
//  C++处理时间的库
#include <chrono>
#include "frame_pack.h"
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
//  导入消息类型文件
#include "agv_interfaces/msg/agv_velo.hpp"
#include "agv_interfaces/msg/agv_odometry.hpp"
#include "agv_interfaces/msg/ret.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace canopen_ {
using namespace std::chrono_literals;
#define DATA_BUF_MAX_LEN 128
typedef union 
{
    uint16_t data_uint16;
    unsigned char data8[2];
}union_uint16;

typedef union 
{
    int16_t data_int16;
    unsigned char data8[2];
}union_int16;

typedef union 
{
    int32_t data_int32;
    unsigned char data8[4];
}union_int32;

//  控制指令返回数据
typedef struct
{
    uint8_t flag;
    uint8_t func;
    int8_t cmd_ret[4];
}control_;
control_ control_ret;

//  速度指令返回数据
typedef struct
{
    uint8_t flag;
    uint8_t func;
    uint8_t velo_ret[4];
}velo_;
velo_ velocity_ret;


//  设置参数指令返回数据
typedef struct
{
    union_uint16 reg;
    uint8_t flag;
    uint8_t func;
    int8_t set_ret[4];
}param_set;
param_set param_set_ret;

//  获取16位参数指令返回数据
typedef struct
{
    union_uint16 reg;
    uint8_t flag;
    uint8_t func;
    uint16_t get_ret[4];
    // union_uint16 get_ret[4];
}param_get16;
param_get16 param_get16_ret;

//  获取32位参数指令返回数据
typedef struct
{
    union_uint16 reg;
    uint8_t flag;
    uint8_t func;
    int32_t get_ret[4];
    // union_int32 get_ret[4];
}param_get32;
param_get32 param_get32_ret;

//  里程计数据
typedef struct
{
    int32_t count_ret[4];
    int16_t velo_ret[4];
}odometry_;
odometry_ odometry_data_;

typedef union 
{
    float data_float;
    uint8_t data8[4];
}union_float;


typedef struct
{
	float accel_x;			/*uinit: m/s2*/
	float accel_y;
	float accel_z;

	float angle_x;			/*uinit: ° (deg)/s*/
	float angle_y;
	float angle_z;
	
	float pitch;			/*uinit: ° (deg)*/
	float roll;
	float yaw;
	
	float quaternion_data0;
	float quaternion_data1;	
	float quaternion_data2;
	float quaternion_data3;
}imu_info_t;
imu_info_t imu_info;


#define SET_ENABLE				0x01
#define SET_DISENABLE			0x02
#define IS_ENABLE				0x03
#define CLEAR_FAULT				0x04
#define IS_FAULT				0x05
#define SET_STOP				0x06
#define STOP_TO_ENABLE			0x07
#define SPEED					0x08
#define SET_PARAM				0x09
#define GET_PARAM				0x0A
#define ODOMETRY_IMU			0x0B

#define SAVE_RW		            0x2009
#define LOCK_METHOD 			0x200F
#define SAVE_RW_S 			    0x2010
#define VELO_SMOOTH_FACTOR 		0x2018
#define ELEC_ERATIO_GAIN 		0x2019
#define ELEC_INTEGRAL 			0x201A
#define FEEDFORWARD_RATIO 		0x201B
#define TORQUE_RATIO 			0x201C
#define VELO_KP 				0x201D
#define VELO_KI 				0x201E
#define VELO_FEEDFORWARD_KF 	0x201F
#define POSI_KP 				0x2020
#define POSI_FEEDFORWARD_KF 	0x2021
#define MOTOR_TEMP 				0x2026
#define IS_MOTOR_MOVE 			0x2027
#define MOTOR_HALL_STATUS		0x2028
#define ERROR_CODE	 			0x603f
#define MOTOR_CONTROL			0x6040
#define MOTOR_STATUS 			0x6041
#define MOTOR_MODE				0x6060
#define MODE_DISPLAY 			0x6061
#define ACTUAL_COUNT			0x6064
#define ACTUAL_VELOCITY			0x606C
#define ACC_TIME	 			0x6083
#define DE_TIME					0x6084


#define IS_REG(REG)		        (((REG) == SAVE_RW) || \
                                 ((REG) == LOCK_METHOD) || \
                                 ((REG) == SAVE_RW_S) || \
								 ((REG) == VELO_SMOOTH_FACTOR) || \
								 ((REG) == ELEC_ERATIO_GAIN) || \
								 ((REG) == ELEC_INTEGRAL) || \
								 ((REG) == FEEDFORWARD_RATIO) || \
								 ((REG) == TORQUE_RATIO) || \
								 ((REG) == VELO_KP) || \
								 ((REG) == VELO_KI) || \
								 ((REG) == VELO_FEEDFORWARD_KF) || \
								 ((REG) == POSI_KP) || \
								 ((REG) == POSI_FEEDFORWARD_KF) || \
                                 ((REG) == MOTOR_TEMP) || \
								 ((REG) == IS_MOTOR_MOVE) || \
								 ((REG) == MOTOR_HALL_STATUS) || \
								 ((REG) == ERROR_CODE) || \
                                 ((REG) == MOTOR_STATUS) || \
                                 ((REG) == MOTOR_CONTROL) || \
								 ((REG) == MOTOR_MODE) || \
								 ((REG) == MODE_DISPLAY) || \
                                 ((REG) == ACTUAL_COUNT) || \
								 ((REG) == ACTUAL_VELOCITY) || \
								 ((REG) == ACC_TIME) || \
								 ((REG) == DE_TIME))

#define IS_CAN_SET(REG)		    (((REG) == SAVE_RW) || \
                                 ((REG) == LOCK_METHOD) || \
                                 ((REG) == SAVE_RW_S) || \
								 ((REG) == VELO_SMOOTH_FACTOR) || \
								 ((REG) == ELEC_ERATIO_GAIN) || \
								 ((REG) == ELEC_INTEGRAL) || \
								 ((REG) == FEEDFORWARD_RATIO) || \
								 ((REG) == TORQUE_RATIO) || \
								 ((REG) == VELO_KP) || \
								 ((REG) == VELO_KI) || \
								 ((REG) == VELO_FEEDFORWARD_KF) || \
								 ((REG) == POSI_KP) || \
								 ((REG) == POSI_FEEDFORWARD_KF) || \
								 ((REG) == ACC_TIME) || \
								 ((REG) == DE_TIME))

#define IS_CAN_GET(REG)		    (((REG) != MOTOR_CONTROL) && \
								 ((REG) != MOTOR_MODE))


class canopen_node: public rclcpp::Node
{
private:
    //  serial库串口对象
    std::shared_ptr<serial::Serial> serial_object;//  串口对象
    std::shared_ptr<std::thread> read_data_thread_;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr cmd_sub{};
    rclcpp::Publisher<agv_interfaces::msg::AgvOdometry>::SharedPtr odometry_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<agv_interfaces::msg::Ret>::SharedPtr ret_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odo_pub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    agv_interfaces::msg::Ret ret_data;
    rclcpp::Time current_time_;

    float odom_x_=0.0, odom_y_=0.0, odom_th_=0.0;

    uint8_t data_rxflag = 0;
    uint8_t data_index = 0;
    uint8_t data_len = 0;
    uint8_t tx_buf[DATA_BUF_MAX_LEN] = {0};
    uint8_t rx_buf[DATA_BUF_MAX_LEN] = {0};
    uint8_t frame_buf[DATA_BUF_MAX_LEN] = {0};
    uint8_t func = 0;

    int64_t select = 0x0A;
    int16_t v1 = 10,v2 = 10,v3 = 10,v4 = 10;
    void _initSerial(void);
    void cmd_callback(const std_msgs::msg::UInt16::SharedPtr cmd_msg);
    void timer_callback(void);
    
    //  下面三个为serial串口库对象的数据处理函数
    void readRawData(void);
    void data_rcv(const uint8_t rxdata);
    void clear_usart1cmd(void);

    void data_analysis(const std::uint8_t* data, const std::uint8_t len);
    void control_analysis(const std::uint8_t* control_retdata, const std::uint8_t len);
    void speed_analysis(const std::uint8_t* speed_retdata, const std::uint8_t len);
    void param_analysis(const std::uint8_t* param_retdata);
    void odometry_imu_analysis(const std::uint8_t* odometry_imu_retdata);
    void odom_pub(float vx, float vth);
public:
    canopen_node(std::string name):Node(name){
        _initSerial();
    }
    ~canopen_node()
    {
        
    }

    //  发送数组
    size_t senddata(const unsigned char* buf, size_t len);
    //  发送字符串
    size_t senddata(const std::string& data);

    
    void control_cmd(uint8_t func);
    void speed_cmd(const int16_t motor1_velo, const int16_t motor2_velo, 
        const int16_t motor3_velo, const int16_t motor4_velo);
    void param_set_cmd(const uint16_t reg, const uint16_t motor1_data, 
        const uint16_t motor2_data, const uint16_t motor3_data, uint16_t motor4_data);
    void param_get_cmd(const uint16_t reg);


    void clearcontrolbuf(void);
    void clearvelobuf(void);
    void clearsetbuf(void);
    void cleargetbuf(void);

    int8_t setenable(void);
    int8_t setdisable(void);
    int8_t isenable(void);
    int8_t clearfault(void);
    int8_t isfault(void);
    int8_t quickstop(void);
    int8_t quickstop_toenable(void);
    int8_t setspeed(const int16_t motor1_velo, const int16_t motor2_velo, 
        const int16_t motor3_velo, const int16_t motor4_velo);


    int8_t set_issave_rw(uint16_t issave);
    int8_t set_lock(uint16_t lock);
    int8_t set_issave_rws(uint16_t issave);
    int8_t set_Vsmooth_factor(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4);
    int8_t set_Eratio_gain(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4);
    int8_t set_Eintegral_gain(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4);
    int8_t set_feedforward_ratio(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4);
    int8_t set_torque_ratio(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4);
    int8_t set_VKp(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4);
    int8_t set_VKi(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4);
    int8_t set_Vfeedforward_Kf(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4);
    int8_t set_PKp(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4);
    int8_t set_Pfeedforward_Kf(uint16_t factor1, uint16_t factor2, uint16_t factor3, uint16_t factor4);
    int8_t set_accelerate_time(uint32_t time1, uint32_t time2, uint32_t time3, uint32_t time4);
    int8_t set_decelerate_time(uint32_t time1, uint32_t time2, uint32_t time3, uint32_t time4);

    // int8_t get_status(void);
    // int8_t get_mode(void);
    int8_t get_count(void);
    // int8_t get_rad(void);
    // int8_t get_distance(void);


    int8_t get_motor_temp(void);
    int8_t get_motor_status(void);
    int8_t get_hall_status(void);
    int8_t get_errorcode(void);
    int8_t get_actual_velocity(void);

    int8_t get_lock(void);
    int8_t get_issave_rws(void);
    int8_t get_Vsmooth_factor(void);
    int8_t get_Eratio_gain(void);
    int8_t get_Eintegral_gain(void);
    int8_t get_feedforward_ratio(void);
    int8_t get_torque_ratio(void);
    int8_t get_VKp(void);
    int8_t get_VKi(void);
    int8_t get_Vfeedforward_Kf(void);
    int8_t get_PKp(void);
    int8_t get_Pfeedforward_Kf(void);
    int8_t get_accelerate_time(void);
    int8_t get_decelerate_time(void);

};



}

