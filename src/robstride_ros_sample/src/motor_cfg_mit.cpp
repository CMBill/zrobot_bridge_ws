#include "motor_ros2/motor_cfg_mit.h"


/**
 * @brief 初始化 CAN 总线套接字
 * 
 * 该方法创建并配置一个 CAN 原始套接字,用于与 RobStride 电机进行通信。
 * 执行以下步骤:
 * 1. 创建 CAN 套接字
 * 2. 获取指定网络接口的索引
 * 3. 绑定套接字到 CAN 接口
 * 
 * @note 如果任何步骤失败,程序将打印错误信息并退出
 * @note 使用类成员变量 iface 指定的网络接口名称
 */
void RobStrideMotor::init_socket()
{
    socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd < 0)
    {
        perror("socket");
        exit(1);
    }

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ);
    if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0)
    {
        perror("ioctl");
        exit(1);
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
    {
        perror("bind");
        exit(1);
    }
}

/**
 * @brief 指令1：电机使能运行
 * 
 * 该方法通过 CAN 总线发送使能命令来启动 RobStride 电机。
 * 构造一个特定格式的 CAN 帧:前 7 字节设置为 0xFF,第 8 字节设置为 0xFC。
 * 
 * CAN 帧格式:
 * - can_id: 电机的主控 ID (motor_id)
 * - can_dlc: 8 字节数据长度
 * - data[0-6]: 0xFF (命令标识)
 * - data[7]: 0xFC (使能命令码)
 * 
 * @note 使用类成员变量 motor_id 作为 CAN 帧 ID
 * @note 如果写入失败,会打印错误信息
 * @note 成功发送后会在控制台输出确认信息
 */
void RobStrideMotor::Enable_Motor() const
{
    struct can_frame frame{};
    frame.can_id = motor_id;
    frame.can_dlc = 8;
    std::memset(&frame, 0xFF, 7);
    frame.data[7] = 0xFC;

    const ssize_t n = write(socket_fd, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame)))
    {
        perror("enable_motor failed");
    }
    else
    {
        std::cout << "[✓] Motor " << motor_id << " enable command sent." << std::endl;
    }
}

/**
 * @brief 指令2：电机停止运行​
 * 
 * 该方法通过 CAN 总线发送失能命令来停止 RobStride 电机。
 * 构造一个特定格式的 CAN 帧:前 7 字节设置为 0xFF,第 8 字节设置为 0xFD。
 * 
 * CAN 帧格式:
 * - can_id: 电机的主控 ID (motor_id)
 * - can_dlc: 8 字节数据长度
 * - data[0-6]: 0xFF (命令标识)
 * - data[7]: 0xFD (失能命令码)
 * 
 * @note 使用类成员变量 motor_id 作为 CAN 帧 ID
 * @note 如果写入失败,会打印错误信息
 * @note 成功发送后会在控制台输出确认信息
 */
void RobStrideMotor::Disable_Motor() const
{
    struct can_frame frame{};
    frame.can_id = motor_id;
    frame.can_dlc = 8;
    std::memset(&frame, 0xFF, 7);
    frame.data[7] = 0xFD;

    const ssize_t n = write(socket_fd, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame)))
    {
        perror("disable_motor failed");
    }
    else
    {
        std::cout << "[✓] Motor " << motor_id << " disable command sent." << std::endl;
    }
}

/**
 * @brief 指令3：电机MIT动态参数
 * 
 * 该方法使用 MIT 模式通过 CAN 总线发送电机控制参数。
 * 将角度、速度、Kp、Kd 和扭矩等浮点数参数编码为特定格式的 CAN 帧数据。
 * 
 * @param Angle 目标角度 (单位:弧度,范围:-12.57 ~ 12.57)
 * @param Velocity 目标速度 (单位:rad/s,范围:-15.0 ~ 15.0)
 * @param Kp 位置比例增益 (范围:0 ~ 5000.0)
 * @param Kd 速度阻尼增益 (范围:0 ~ 100)
 * @param Torque 前馈扭矩 (单位:N·m,范围:-120 ~ 120)
 * 
 * CAN 帧数据格式 (8 字节):
 * - data[0-1]: 角度 (16 位)
 * - data[2-3]: 速度 (12 位) + Kp 高 4 位
 * - data[4]: Kp 低 8 位
 * - data[5-6]: Kd (12 位) + 扭矩高 8 位
 * - data[7]: 扭矩低 4 位
 * 
 * @note 所有参数会被映射到指定位宽的无符号整数
 * @note 如果写入失败,会打印错误信息
 * @note 成功发送后会在控制台输出确认信息
 */
void RobStrideMotor::Set_MIT_Dynamic_Param(const float Angle, const float Velocity,
                                           const float Kp, const float Kd, const float Torque) const
{
    struct can_frame frame{};
    frame.can_id = motor_id;
    frame.can_dlc = 8;

    const int angle_mapped = float_to_uint(Angle, -12.57, 12.57, 16);
    const int velocity_mapped = float_to_uint(Velocity, -15.0, 15.0, 12);
    const int Kp_mapped = float_to_uint(Kp, 0, 5000.0, 12);
    const int Kd_mapped = float_to_uint(Kd, 0, 100, 12);
    const int torque_mapped = float_to_uint(Torque, -120, 120, 12);

    frame.data[0] = angle_mapped >> 8 & 0xFF;
    frame.data[1] = angle_mapped & 0xFF;
    frame.data[2] = velocity_mapped >> 4 & 0xFF;
    frame.data[3] = (velocity_mapped & 0x0F << 4) | (Kp_mapped >> 8 & 0xFF);
    frame.data[4] = Kp_mapped & 0xFF;
    frame.data[5] = Kd_mapped >> 4 & 0xFF;
    frame.data[6] = (Kd_mapped & 0x0F << 4) | (torque_mapped >> 4 & 0xFF);
    frame.data[7] = torque_mapped & 0xFF;

    const ssize_t n = write(socket_fd, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame)))
    {
        perror("Set MIT ctrl params failed");
    } else
    {
        std::cout << "[✓] Motor " << motor_id << " MIT ctrl params sent." << std::endl;
    }
}

/**
 * @brief 指令4：设置零点（非位置模式）
 * 
 * 该方法通过 CAN 总线发送零点设置命令,将电机当前位置设置为零点参考位置。
 * 构造一个特定格式的 CAN 帧:前 7 字节设置为 0xFF,第 8 字节设置为 0xFE。
 * 
 * CAN 帧格式:
 * - can_id: 电机的主控 ID (motor_id)
 * - can_dlc: 8 字节数据长度
 * - data[0-6]: 0xFF (命令标识)
 * - data[7]: 0xFE (设置零点命令码)
 * 
 * @note 使用类成员变量 motor_id 作为 CAN 帧 ID
 * @note 如果写入失败,会打印错误信息
 * @note 成功发送后会在控制台输出确认信息
 * @warning 设置零点操作会改变电机的位置参考基准,请在确认电机位置正确后执行
 */
void RobStrideMotor::Set_Zero_Point() const
{
    struct can_frame frame{};
    frame.can_id = motor_id;
    frame.can_dlc = 8;
    std::memset(&frame, 0xFF, 7);
    frame.data[7] = 0xFE;

    const ssize_t n = write(socket_fd, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame)))
    {
        perror("set zero failed");
    }
    else
    {
        std::cout << "[✓] Motor " << motor_id << " zero point set command sent." << std::endl;
    }
}

/**
 * @brief 指令5：清除错误及读取异常状态
 *
 * 该方法通过 CAN 总线发送命令以获取电机当前错误状态或清除已发生的异常。
 * 构造一个特定格式的 CAN 帧:前 6 字节设置为 0xFF,第 7 字节根据操作类型设置,
 * 第 8 字节设置为 0xFB。
 *
 * @param clear_flag 操作标志
 *                   - true: 仅获取错误状态值 (data[6] = 0x00)
 *                   - false: 清除错误状态 (data[6] = 0xFF)
 *
 * CAN 帧格式:
 * - can_id: 电机的主控 ID (motor_id)
 * - can_dlc: 8 字节数据长度
 * - data[0-5]: 0xFF (命令标识)
 * - data[6]: 0x00 (获取错误) 或 0xFF (清除错误)
 * - data[7]: 0xFB (错误状态命令码)
 *
 * @note 使用类成员变量 motor_id 作为 CAN 帧 ID
 * @note 如果写入失败,会打印错误信息
 * @note 成功发送后会在控制台输出确认信息
 * @note 电机会通过 CAN 响应返回具体的错误代码
 */
void RobStrideMotor::Get_Error_Status(const bool clear_flag) const
{
    struct can_frame frame{};
    frame.can_id = motor_id;
    frame.can_dlc = 8;
    std::memset(&frame, 0xFF, 6);
    frame.data[7] = 0xFB;

    if (clear_flag)
    {
        frame.data[6] = 0x00; // 获取错误值
    } else
    {
        frame.data[6] = 0xFF; // 消除异常
    }

    const ssize_t n = write(socket_fd, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame)))
    {
        perror("Get_Error_Status failed");
    }
    else
    {
        std::cout << "[✓] Motor " << motor_id << " Get_Error_Status command sent." << std::endl;
    }
}

/**
 * @brief 设置电机运行模式
 * 
 * 该方法通过 CAN 总线发送命令来设置 RobStride 电机的运行模式。
 * 构造一个特定格式的 CAN 帧:前 6 字节设置为 0xFF,第 7 字节设置为模式值,
 * 第 8 字节设置为 0xFC。
 * 
 * @param mode 电机运行模式 (MotorMode 枚举类型)
 *             具体模式值取决于 MotorMode 枚举定义
 * 
 * CAN 帧格式:
 * - can_id: 电机的主控 ID (motor_id)
 * - can_dlc: 8 字节数据长度
 * - data[0-5]: 0xFF (命令标识)
 * - data[6]: 模式值 (根据 MotorMode 枚举转换)
 * - data[7]: 0xFC (设置模式命令码)
 * 
 * @note 使用类成员变量 motor_id 作为 CAN 帧 ID
 * @note 如果写入失败,会打印错误信息
 * @note 成功发送后会在控制台输出确认信息
 */
void RobStrideMotor::Set_Motor_Mode(MotorMode mode) const
{
    struct can_frame frame{};
    frame.can_id = motor_id;
    frame.can_dlc = 8;
    std::memset(&frame, 0xFF, 6);
    frame.data[6] = static_cast<ssize_t>(mode);
    frame.data[7] = 0xFC;

    const ssize_t n = write(socket_fd, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame)))
    {
        perror("Set_Motor_Mode failed");
    }
    else
    {
        std::cout << "[✓] Motor " << motor_id << " set mode to" << static_cast<int>(mode) << " command sent." << std::endl;
    }
}

/**
 * @brief 指令7：修改电机CANID
 *
 * 该方法通过 CAN 总线发送命令来修改 RobStride 电机的 ID。
 * 使用旧 ID 发送 CAN 帧,帧中包含新的 ID 值,电机接收后会更新自身 ID。
 *
 * @param motor_id_new 新的电机 ID (范围:0-255)
 *
 * CAN 帧格式:
 * - can_id: 电机的旧 ID (motor_id_old)
 * - can_dlc: 8 字节数据长度
 * - data[0-5]: 0xFF (命令标识)
 * - data[6]: 新的电机 ID
 * - data[7]: 0xFA (设置 ID 命令码)
 *
 * @note 该方法会更新类成员变量 motor_id 为新值
 * @note 发送时使用旧 ID,电机收到命令后会应用新 ID
 * @note 如果写入失败,会打印错误信息
 * @note 成功发送后会在控制台输出确认信息
 * @warning ID 修改后立即生效,后续通信需使用新 ID
 */
void RobStrideMotor::Set_Motor_ID(const uint8_t motor_id_new)
{
    const uint8_t motor_id_old = motor_id;
    motor_id = motor_id_new;

    struct can_frame frame{};
    frame.can_id = motor_id_old;
    frame.can_dlc = 8;
    std::memset(&frame, 0xFF, 6);
    frame.data[6] = motor_id_new;
    frame.data[7] = 0xFA;

    const ssize_t n = write(socket_fd, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame)))
    {
        perror("Set_Motor_ID failed");
    }
    else
    {
        std::cout << "[✓] Motor " << motor_id_old << " id set to" << motor_id << "  command sent." << std::endl;
    }
}

/**
 * @brief 指令8：修改电机协议：切换电机协议，重新上电生效
 *
 * 该方法通过 CAN 总线发送命令来设置 RobStride 电机的通信协议类型。
 * 构造一个特定格式的 CAN 帧:前 6 字节设置为 0xFF,第 7 字节设置为协议类型值,
 * 第 8 字节设置为 0xFD。
 *
 * @param communication_protocol 通信协议类型 (MotorCommunicationProtocol 枚举类型)
 *                               具体协议类型取决于 MotorCommunicationProtocol 枚举定义
 *
 * CAN 帧格式:
 * - can_id: 电机的主控 ID (motor_id)
 * - can_dlc: 8 字节数据长度
 * - data[0-5]: 0xFF (命令标识)
 * - data[6]: 协议类型值 (根据 MotorCommunicationProtocol 枚举转换)
 * - data[7]: 0xFD (设置通信协议命令码)
 *
 * @note 使用类成员变量 motor_id 作为 CAN 帧 ID
 * @note 如果写入失败,会打印错误信息
 * @note 成功发送后会在控制台输出确认信息
 * @warning 更改通信协议后,需确保后续通信使用对应的协议格式
 */
void RobStrideMotor::Set_Motor_Communication_Protocol(MotorCommunicationProtocol communication_protocol) const
{
    struct can_frame frame{};
    frame.can_id = motor_id;
    frame.can_dlc = 8;
    std::memset(&frame, 0xFF, 6);
    frame.data[6] = static_cast<uint8_t>(communication_protocol);
    frame.data[7] = 0xFD;

    const ssize_t n = write(socket_fd, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame)))
    {
        perror("Set_Motor_Communication_Protocol failed");
    }
    else
    {
        std::cout << "[✓] Motor " << motor_id << " set protocol to" << static_cast<int>(communication_protocol)
                  << "  command sent." << std::endl;
    }
}

/**
 * @brief 指令9：修改主机canid
 *
 * 该方法通过 CAN 总线发送命令来设置 RobStride 电机的主控 ID。
 * 构造一个特定格式的 CAN 帧:前 6 字节设置为 0xFF,第 7 字节设置为新的主控 ID,
 * 第 8 字节设置为 0x01。
 *
 * @param master_id_new 新的主控 ID (范围:0-255)
 *
 * CAN 帧格式:
 * - can_id: 电机的 ID (motor_id)
 * - can_dlc: 8 字节数据长度
 * - data[0-5]: 0xFF (命令标识)
 * - data[6]: 新的主控 ID
 * - data[7]: 0x01 (设置主控 ID 命令码)
 *
 * @note 该方法会更新类成员变量 master_id 为新值
 * @note 使用类成员变量 motor_id 作为 CAN 帧 ID
 * @note 如果写入失败,会打印错误信息
 * @note 成功发送后会在控制台输出确认信息
 */
void RobStrideMotor::Set_Master_ID(const uint8_t master_id_new)
{
    master_id = master_id_new;

    struct can_frame frame{};
    frame.can_id = motor_id;
    frame.can_dlc = 8;
    std::memset(&frame, 0xFF, 6);
    frame.data[6] = master_id_new;
    frame.data[7] = 0x01;

    const ssize_t n = write(socket_fd, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame)))
    {
        perror("Set_Mater_ID failed");
    }
    else
    {
        std::cout << "[✓] Motor " << motor_id << " master id set to" << master_id_new << "  command sent." << std::endl;
    }
}

/**
 * @brief 指令10：位置模式控制指令
 *
 * 该方法通过 CAN 总线发送角度控制命令,控制电机运动到指定角度位置。
 * 使用扩展 CAN ID 格式,将命令类型和电机 ID 组合编码。
 *
 * @param Angle 目标角度 (单位:弧度)
 * @param Velocity 最大运动速度 (单位:rad/s)
 *
 * CAN 帧格式:
 * - can_id: (0x1 << 8) | motor_id (命令类型 0x1 表示位置控制)
 * - can_dlc: 8 字节数据长度
 * - data[0-3]: 目标角度 (float,小端序)
 * - data[4-7]: 最大速度 (float,小端序)
 *
 * @note 使用类成员变量 motor_id 作为 CAN 帧 ID 的一部分
 * @note 角度和速度以 IEEE 754 单精度浮点数格式直接复制到数据帧
 * @note 如果写入失败,会打印错误信息
 * @note 成功发送后会在控制台输出确认信息
 */
void RobStrideMotor::Set_Motor_Angle(const float Angle, const float Velocity) const
{
    struct can_frame frame{};
    frame.can_id = 0x1 << 8 | motor_id;
    frame.can_dlc = 8;

    std::memcpy(&frame.data[0], &Angle, 4);
    std::memcpy(&frame.data[4], &Velocity, 4);

    const ssize_t n = write(socket_fd, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame)))
    {
        perror("Set_Motor_Angle failed");
    }
    else
    {
        std::cout << "[✓] Motor " << motor_id << " Set_Motor_Angle command sent." << std::endl;
    }
}

/**
 * @brief 指令11：速度模式控制指令
 *
 * 该方法通过 CAN 总线发送速度控制命令,控制电机以指定速度运动。
 * 使用扩展 CAN ID 格式,将命令类型和电机 ID 组合编码。
 *
 * @param Velocity 目标速度 (单位:rad/s)
 * @param I_Limit 电流限制 (单位:A)
 *
 * CAN 帧格式:
 * - can_id: (0x2 << 8) | motor_id (命令类型 0x2 表示速度控制)
 * - can_dlc: 8 字节数据长度
 * - data[0-3]: 目标速度 (float,小端序)
 * - data[4-7]: 电流限制 (float,小端序)
 *
 * @note 使用类成员变量 motor_id 作为 CAN 帧 ID 的一部分
 * @note 速度和电流限制以 IEEE 754 单精度浮点数格式直接复制到数据帧
 * @note 如果写入失败,会打印错误信息
 * @note 成功发送后会在控制台输出确认信息
 */
void RobStrideMotor::Set_Motor_Velocity(const float Velocity, const float I_Limit) const
{
    struct can_frame frame{};
    frame.can_id = 0x2 << 8 | motor_id;
    frame.can_dlc = 8;

    std::memcpy(&frame.data[0], &Velocity, 4);
    std::memcpy(&frame.data[4], &I_Limit, 4);

    const ssize_t n = write(socket_fd, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame)))
    {
        perror("Set_Motor_Velocity failed");
    }
    else
    {
        std::cout << "[✓] Motor " << motor_id << " Set_Motor_Velocity command sent." << std::endl;
    }
}

/**
 * @brief 浮点数转无符号整数
 * 
 * 将指定范围内的浮点数线性映射转换为指定位宽的无符号整数。
 * 该方法用于将电机参数(如角度、速度等)从浮点数格式编码为整数格式,
 * 以便通过 CAN 总线发送。
 * 
 * @param x 待转换的浮点数值
 * @param x_min 浮点数的最小值(映射到 0)
 * @param x_max 浮点数的最大值(映射到 2^bits - 1)
 * @param bits 输出整数的位宽(如 12 位、16 位等)
 * 
 * @return 转换后的无符号整数值,范围为 [0, 2^bits - 1]
 * 
 * @note 如果输入值超出 [x_min, x_max] 范围,会被限制在该范围内
 * @note 映射公式: output = (x - x_min) / (x_max - x_min) * (2^bits - 1)
 * 
 * @example
 * float_to_uint(0.5, 0.0, 1.0, 12) 返回 2048 (0x800)
 */
int RobStrideMotor::float_to_uint(float x, const float x_min, const float x_max, const int bits)
{
    const float span = x_max - x_min;
    if (x > x_max) { x = x_max;}
    else if (x < x_min) { x = x_min;}
    return static_cast<int>(((x - x_min) * static_cast<float>((1 << bits) - 1)) / span);
}