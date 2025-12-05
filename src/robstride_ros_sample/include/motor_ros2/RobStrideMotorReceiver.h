//
// Created by bill on 2025 12 05.
//

#ifndef RS_MOTOR_ROS2_ROBSTRIDEMOTORRECEIVE_H
#define RS_MOTOR_ROS2_ROBSTRIDEMOTORRECEIVE_H

#include <iostream>
#include <linux/can.h>
#include <vector>
#include <optional>
#include <functional>
#include <chrono>

#include <map>
#include <atomic>
#include <mutex>
#include <thread>

using ReceiveResult = std::optional<std::tuple<uint8_t, uint16_t, uint8_t, std::vector<uint8_t>>>;

class RobStrideMotorReceiver
{
public:
    struct FeedbackData {
            float position;
            float velocity;
            float torque;
            float temperature;
            uint8_t error_code;
            std::chrono::steady_clock::time_point timestamp;
        };
    using FeedbackCallback = std::function<void(uint8_t motor_id, const FeedbackData&)>;

    RobStrideMotorReceiver(const std::string& can_interface);
    ~RobStrideMotorReceiver();

    // 注册电机回调函数
    void register_motor(uint8_t motor_id, FeedbackCallback callback);
    void unregister_motor(uint8_t motor_id);

    // 启动/停止接收线程
    void start();
    void stop();


private:

    void receive_thread_func();
    void parse_and_dispatch(const can_frame& frame);

    int socket_fd_;
    std::string iface_;
    std::atomic<bool> running_;
    std::thread receive_thread_;

    std::mutex callbacks_mutex_;
    std::map<uint8_t, FeedbackCallback> motor_callbacks_;

};


#endif //RS_MOTOR_ROS2_ROBSTRIDEMOTORRECEIVE_H