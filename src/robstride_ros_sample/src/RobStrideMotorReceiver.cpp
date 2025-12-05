//
// Created by bill on 2025 12 05.
//

#include "../include/motor_ros2/RobStrideMotorReceiver.h"

void RobStrideMotorReceiver::receive_thread_func() {
    can_frame frame;
    while (running_) {
        ssize_t nbytes = read(socket_fd_, &frame, sizeof(frame));
        if (nbytes > 0 && (frame.can_id & CAN_EFF_FLAG)) {
            parse_and_dispatch(frame);
        }
    }
}

void RobStrideMotorReceiver::parse_and_dispatch(const can_frame& frame) {
    uint32_t can_id = frame.can_id & CAN_EFF_MASK;
    uint8_t communication_type = (can_id >> 24) & 0xFF;

    // 从data中提取电机ID(根据您的协议)
    uint8_t motor_id = frame.data[0]; // 假设在data[0]

    if (communication_type == 0x02) { // 电机反馈帧
        FeedbackData data;
        // 解析frame.data填充data结构
        // ...existing parsing code...

        std::lock_guard<std::mutex> lock(callbacks_mutex_);
        auto it = motor_callbacks_.find(motor_id);
        if (it != motor_callbacks_.end()) {
            it->second(motor_id, data);
        }
    }
}

