//
// Created by bill on 2025 12 05.
//

#ifndef RS_MOTOR_ROS2_ROBSTRIDEMOTORRECEIVER_H
#define RS_MOTOR_ROS2_ROBSTRIDEMOTORRECEIVER_H

#include <iostream>
#include <cstring>
#include <string>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>

/**
 * @brief 电机反馈数据结构
 */
struct MotorFeedback
{
    uint8_t motor_id;           // 电机 ID
    float position;             // 位置 (弧度)
    float velocity;             // 速度 (rad/s)
    float torque;               // 扭矩 (N·m)
    float temperature;          // 温度 (℃)
    uint16_t error_code;        // 错误代码
    std::chrono::steady_clock::time_point timestamp;  // 时间戳
    bool is_valid;              // 数据有效性标志

    MotorFeedback() 
        : motor_id(0), position(0.0f), velocity(0.0f), 
          torque(0.0f), temperature(0.0f), error_code(0),
          timestamp(std::chrono::steady_clock::now()), is_valid(false) {}
};

/**
 * @brief RobStride 电机反馈接收器类
 * 
 * 该类负责从 CAN 总线接收电机反馈消息,并为每个电机提供非阻塞的消息查询接口。
 * 支持多电机并发使用,线程安全。
 */
class RobStrideMotorReceiver
{
public:
    /**
     * @brief 构造函数
     * 
     * @param can_interface CAN 接口名称 (如 "can0")
     * @param master_id 主机 ID (默认 0x00)
     */
    explicit RobStrideMotorReceiver(std::string  can_interface, uint8_t master_id = 0x00);

    /**
     * @brief 析构函数,停止接收线程并关闭套接字
     */
    ~RobStrideMotorReceiver();

    /**
     * @brief 启动接收线程
     * 
     * @return true 启动成功, false 启动失败
     */
    bool Start();

    /**
     * @brief 停止接收线程
     */
    void Stop();

    /**
     * @brief 获取指定电机的最新反馈数据 (非阻塞)
     * 
     * @param motor_id 电机 ID
     * @param feedback 输出参数,存储反馈数据
     * @param timeout_ms 超时时间(毫秒),默认 100ms
     * @return true 获取到有效数据, false 未获取到或超时
     */
    bool GetMotorFeedback(uint8_t motor_id, MotorFeedback& feedback, int timeout_ms = 100);

    /**
     * @brief 清除指定电机的反馈数据缓存
     * 
     * @param motor_id 电机 ID
     */
    void ClearMotorFeedback(uint8_t motor_id);

    /**
     * @brief 检查接收器是否正在运行
     * 
     * @return true 正在运行, false 未运行
     */
    bool IsRunning() const { return running_.load(); }

private:
    std::string iface_;                                     // CAN 接口名称
    uint8_t master_id_;                                     // 主机 ID
    int socket_fd_;                                         // CAN 套接字文件描述符

    std::atomic<bool> running_;                             // 运行状态标志
    std::thread receive_thread_;                            // 接收线程

    std::map<uint8_t, MotorFeedback> feedback_buffer_;      // 反馈数据缓冲区 (motor_id -> feedback)
    std::mutex buffer_mutex_;                               // 缓冲区互斥锁

    /**
     * @brief 初始化 CAN 套接字
     * 
     * @return true 初始化成功, false 初始化失败
     */
    bool InitSocket();

    /**
     * @brief 接收线程主函数
     */
    void ReceiveLoop();

    /**
     * @brief 解析 CAN 帧并提取电机反馈数据
     * 
     * @param frame CAN 帧
     * @param feedback 输出参数,存储解析后的反馈数据
     * @return true 解析成功, false 解析失败
     */
    bool ParseFeedback(const struct can_frame& frame, MotorFeedback& feedback);

    /**
     * @brief 将无符号整数转换为浮点数
     * 
     * @param x 待转换的无符号整数
     * @param x_min 浮点数的最小值
     * @param x_max 浮点数的最大值
     * @param bits 整数的位宽
     * @return 转换后的浮点数
     */
    static float uint_to_float(int x, float x_min, float x_max, int bits);
};

#endif //RS_MOTOR_ROS2_ROBSTRIDEMOTORRECEIVER_H