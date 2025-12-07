//
// Created by bill on 2025 12 03.
//
#ifndef RS_MOTOR_ROS2_MOTOR_CFG_MIT_H
#define RS_MOTOR_ROS2_MOTOR_CFG_MIT_H

#include <iostream>
#include <cstring>
#include <memory>
#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

// 前向声明
class RobStrideMotorReceiver;
struct MotorFeedback;


enum class MotorMode : uint8_t
{
    MIT_MODE = 0x00,
    POSITION_MODE = 0x01,
    VELOCITY_MODE = 0x02,
};
enum class MotorCommunicationProtocol : uint8_t
{
    ROBSTRIDE_PROTOCOL = 0x00,
    CANOPEN_PROTOCOL = 0x01,
    MIT_PROTOCOL = 0x02,
};


class RobStrideMotor
{
public:
    RobStrideMotor(uint8_t motor_id, std::string can_interface, uint8_t master_id,
                   std::shared_ptr<RobStrideMotorReceiver> receiver = nullptr)
        : motor_id(motor_id),
          iface(std::move(can_interface)),
          master_id(master_id),
          receiver_(receiver)
    {
        init_socket();
    }

    void Enable_Motor() const;
    void Disable_Motor() const;
    void Set_MIT_Dynamic_Param(float Angle, float Velocity, float Kp, float Kd, float Torque) const;
    void Set_Zero_Point() const;
    void Get_Error_Status(bool clear_flag = false) const;
    void Set_Motor_Mode(MotorMode mode = MotorMode::MIT_MODE) const;
    void Set_Motor_ID(uint8_t motor_id_new);
    void Set_Motor_Communication_Protocol(MotorCommunicationProtocol communication_protocol) const;
    void Set_Master_ID(uint8_t master_id_new);
    void Set_Motor_Angle(float Angle, float Velocity) const;
    void Set_Motor_Velocity(float Velocity, float I_Limit) const;

    // 设置接收器
    void SetReceiver(std::shared_ptr<RobStrideMotorReceiver> receiver);
    
    // 获取最新反馈数据
    bool GetLatestFeedback(MotorFeedback& feedback, int timeout_ms = 50) const;

private:
    
    float position_ = 0.0f;
    float velocity_ = 0.0f;
    float torque_ = 0.0f;
    float temperature_ = 0.0f;

    std::shared_ptr<RobStrideMotorReceiver> receiver_;

    uint8_t motor_id;

    std::string iface;
    uint8_t master_id = 0x00;
    int socket_fd = -1;

    void init_socket();
    static int float_to_uint(float x, float x_min, float x_max, int bits);
    
    // 发送消息后自动获取反馈
    bool send_and_get_feedback(const struct can_frame& frame) const;

};

#endif //RS_MOTOR_ROS2_MOTOR_CFG_MIT_H