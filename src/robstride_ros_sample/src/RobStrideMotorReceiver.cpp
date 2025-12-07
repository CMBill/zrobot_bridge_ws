//
// Created by bill on 2025 12 05.
//

#include <utility>

#include "../include/motor_ros2/RobStrideMotorReceiver.h"

RobStrideMotorReceiver::RobStrideMotorReceiver(std::string  can_interface, uint8_t master_id)
    : iface_(std::move(can_interface)), master_id_(master_id), socket_fd_(-1), running_(false)
{
}

RobStrideMotorReceiver::~RobStrideMotorReceiver()
{
    Stop();
    if (socket_fd_ >= 0)
    {
        close(socket_fd_);
    }
}

bool RobStrideMotorReceiver::InitSocket()
{
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0)
    {
        perror("RobStrideMotorReceiver: socket creation failed");
        return false;
    }

    // 设置为非阻塞模式
    int flags = fcntl(socket_fd_, F_GETFL, 0);
    if (flags == -1)
    {
        perror("RobStrideMotorReceiver: fcntl F_GETFL failed");
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
    if (fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) == -1)
    {
        perror("RobStrideMotorReceiver: fcntl F_SETFL failed");
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, iface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
    {
        perror("RobStrideMotorReceiver: ioctl failed");
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
    {
        perror("RobStrideMotorReceiver: bind failed");
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    std::cout << "[✓] RobStrideMotorReceiver initialized on " << iface_ 
              << " with master_id: 0x" << std::hex << static_cast<int>(master_id_) 
              << std::dec << std::endl;

    return true;
}

bool RobStrideMotorReceiver::Start()
{
    if (running_.load())
    {
        std::cerr << "[!] RobStrideMotorReceiver already running" << std::endl;
        return false;
    }

    if (!InitSocket())
    {
        return false;
    }

    running_.store(true);
    receive_thread_ = std::thread(&RobStrideMotorReceiver::ReceiveLoop, this);

    std::cout << "[✓] RobStrideMotorReceiver started" << std::endl;
    return true;
}

void RobStrideMotorReceiver::Stop()
{
    if (!running_.load())
    {
        return;
    }

    running_.store(false);

    if (receive_thread_.joinable())
    {
        receive_thread_.join();
    }

    std::cout << "[✓] RobStrideMotorReceiver stopped" << std::endl;
}

void RobStrideMotorReceiver::ReceiveLoop()
{
    struct can_frame frame{};
    
    while (running_.load())
    {
        ssize_t nbytes = read(socket_fd_, &frame, sizeof(frame));
        
        if (nbytes < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                // 非阻塞模式下没有数据,短暂休眠避免 CPU 占用过高
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                continue;
            }
            else
            {
                perror("RobStrideMotorReceiver: read failed");
                break;
            }
        }
        else if (nbytes == sizeof(frame))
        {
            // 检查是否是目标主机 ID 的消息
            if (frame.can_id == master_id_)
            {
                MotorFeedback feedback;
                if (ParseFeedback(frame, feedback))
                {
                    std::lock_guard<std::mutex> lock(buffer_mutex_);
                    feedback_buffer_[feedback.motor_id] = feedback;
                }
            }
        }
    }
}

bool RobStrideMotorReceiver::ParseFeedback(const struct can_frame& frame, MotorFeedback& feedback)
{
    if (frame.can_dlc < 8)
    {
        return false;
    }

    // 第一个字节是电机 ID
    feedback.motor_id = frame.data[0];

    // 根据 RobStride 协议解析反馈数据
    // 假设反馈格式为: [motor_id][position_h][position_l][velocity_h][velocity_l][torque_h][torque_l][temp/error]
    // 实际格式需要根据您的电机协议文档调整
    
    // 位置 (16位, -12.57 ~ 12.57 rad)
    int pos_raw = (frame.data[1] << 8) | frame.data[2];
    feedback.position = uint_to_float(pos_raw, -12.57f, 12.57f, 16);

    // 速度 (12位, -15.0 ~ 15.0 rad/s)
    int vel_raw = ((frame.data[3] << 4) | (frame.data[4] >> 4)) & 0xFFF;
    feedback.velocity = uint_to_float(vel_raw, -15.0f, 15.0f, 12);

    // 扭矩 (12位, -120 ~ 120 N·m)
    int torque_raw = (((frame.data[4] & 0x0F) << 8) | frame.data[5]) & 0xFFF;
    feedback.torque = uint_to_float(torque_raw, -120.0f, 120.0f, 12);

    // 温度或错误代码 (根据协议)
    feedback.temperature = static_cast<float>(frame.data[6]);
    feedback.error_code = frame.data[7];

    feedback.timestamp = std::chrono::steady_clock::now();
    feedback.is_valid = true;

    return true;
}

bool RobStrideMotorReceiver::GetMotorFeedback(uint8_t motor_id, MotorFeedback& feedback, int timeout_ms)
{
    auto start_time = std::chrono::steady_clock::now();
    
    while (true)
    {
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            auto it = feedback_buffer_.find(motor_id);
            if (it != feedback_buffer_.end() && it->second.is_valid)
            {
                feedback = it->second;
                return true;
            }
        }

        // 检查超时
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();
        
        if (elapsed >= timeout_ms)
        {
            return false;
        }

        // 短暂休眠,避免忙等待
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void RobStrideMotorReceiver::ClearMotorFeedback(uint8_t motor_id)
{
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    auto it = feedback_buffer_.find(motor_id);
    if (it != feedback_buffer_.end())
    {
        it->second.is_valid = false;
    }
}

float RobStrideMotorReceiver::uint_to_float(int x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (static_cast<float>(x) * span) / static_cast<float>((1 << bits) - 1) + offset;
}
