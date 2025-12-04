#include "motor_ros2/motor_cfg.h"

/**
 * @brief 初始化 CAN 总线套接字
 * 
 * 该方法创建并配置 CAN 套接字，用于与 RobStride 电机进行通信。
 * 主要完成以下操作：
 * 1. 创建原始 CAN 套接字
 * 2. 获取指定网络接口的索引
 * 3. 绑定套接字到 CAN 接口
 * 4. 设置 CAN 过滤器，只接收来自指定电机 ID 的扩展帧
 * 
 * @note 如果初始化失败，程序将直接退出
 * @throw 通过 perror 输出错误信息并调用 exit(1) 终止程序
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

    if (bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("bind");
        exit(1);
    }

    struct can_filter rfilter[1];
    rfilter[0].can_id   = (master_id << 8) | CAN_EFF_FLAG;   // Bit8~Bit15 放电机ID，高位扩展帧标志
    rfilter[0].can_mask = (0xFF << 8) | CAN_EFF_FLAG;          // 只匹配 Bit8~Bit15 + 扩展帧标志

    if (setsockopt(socket_fd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0) 
    {
        perror("setsockopt filter");
        exit(1);
    }

}

/**
 * @brief 接收并解析电机状态帧
 * 
 * 该方法从 CAN 总线接收电机反馈的状态帧，并根据通信类型解析数据。
 * 支持两种主要的通信类型：
 * 1. 电机反馈帧 (Communication_Type_MotorRequest = 0x02)：包含位置、速度、转矩和温度信息
 * 2. 参数反馈帧 (communication_type = 17)：包含电机参数的读取结果
 * 
 * @throws std::runtime_error 当未接收到帧、数据大小不足或通信类型无效时抛出异常
 * 
 * @note 解析的数据会更新类成员变量：
 *       - position_, velocity_, torque_, temperature_ (电机反馈帧)
 *       - drw 结构体中的各参数 (参数反馈帧)
 */
void RobStrideMotor::receive_status_frame()
{
    auto result = receive();
    if (!result)
    {
        throw std::runtime_error("No frame received.");
    }

    auto [communication_type, extra_data, host_id, data] = *result;

    // uint8_t status_mode = (extra_data >> 14) & 0x03;
    // uint8_t status_uncalibrated = (extra_data >> 13) & 0x01;
    // uint8_t status_hall_encoder_fault = (extra_data >> 12) & 0x01;
    // uint8_t status_magnetic_encoder_fault = (extra_data >> 11) & 0x01;
    // uint8_t status_overtemperature = (extra_data >> 10) & 0x01;
    // uint8_t status_overcurrent = (extra_data >> 9) & 0x01;
    // uint8_t status_undervoltage = (extra_data >> 8) & 0x01;
    // uint8_t device_id = (extra_data >> 0) & 0xFF;

    if (data.size() < 8)
    {
        throw std::runtime_error("Data size too small");
    }

    std::cout << "communication_type: " << static_cast<int>(communication_type) << std::endl;
    if (communication_type == Communication_Type_MotorRequest)
    {
        // 解析数据：高字节在前（大端序）
        uint16_t position_u16 = (data[0] << 8) | data[1];
        uint16_t velocity_u16 = (data[2] << 8) | data[3];
        uint16_t torque_i16 = (data[4] << 8) | data[5];
        uint16_t temperature_u16 = (data[6] << 8) | data[7];

        // 转换成物理量
        position_ = ((static_cast<float>(position_u16) / 32767.0f) - 1.0f) * (ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type)).position);
        velocity_ = ((static_cast<float>(velocity_u16) / 32767.0f) - 1.0f) * (ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type)).velocity);
        torque_ = ((static_cast<float>(torque_i16) / 32767.0f) - 1.0f) * (ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type)).torque);
        temperature_ = static_cast<float>(temperature_u16) * 0.1f;

    }
    else if(communication_type == 17)
    {
        params.data = uint8_t(data[4]);
        std::cout << params.data << std::endl;
        params.index = 0X7005;
        for (int index_num = 0; index_num <= 13; index_num++)
        {
            if ((data[1]<<8|data[0]) == Index_List[index_num])
                switch(index_num)
                {
                    case 0:
                        drw.run_mode.data = uint8_t(data[4]);
                        std::cout << "mode data: " << static_cast<int>(data[4]) << std::endl;;
                        break;
                    case 1:
                        drw.iq_ref.data = Byte_to_float(data);
                        break;
                    case 2:
                        drw.spd_ref.data = Byte_to_float(data);
                        break;
                    case 3:
                        drw.imit_torque.data = Byte_to_float(data);
                        break;
                    case 4:
                        drw.cur_kp.data = Byte_to_float(data);
                        break;
                    case 5:
                        drw.cur_ki.data = Byte_to_float(data);
                        break;
                    case 6:
                        drw.cur_filt_gain.data = Byte_to_float(data);
                        break;
                    case 7:
                        drw.loc_ref.data = Byte_to_float(data);
                        break;
                    case 8:
                        drw.limit_spd.data = Byte_to_float(data);
                        break;
                    case 9:
                        drw.limit_cur.data = Byte_to_float(data);
                        break;	
                    case 10:
                        drw.mechPos.data = Byte_to_float(data);
                        break;	
                    case 11:
                        drw.iqf.data = Byte_to_float(data);
                        break;	
                    case 12:
                        drw.mechVel.data =Byte_to_float(data);
                        break;	
                    case 13:
                        drw.VBUS.data = Byte_to_float(data);
                        break;	
                }
		}

    }
    else
    {
        throw std::runtime_error("Invalid communication type");
    }
}

/**
 * @brief 设置 RobStride 电机的单个参数
 * 
 * 该方法通过 CAN 总线向电机发送参数设置命令，支持两种参数类型：
 * 1. 普通参数 ('p')：如速度、位置、电流等浮点型参数
 * 2. 模式参数 ('j')：如控制模式等整型参数
 * 
 * @param Index 参数索引地址（如 0x7005-运行模式, 0x7006-Iq指令等）
 * @param Value 参数值
 * @param Value_mode 参数类型标志
 *                   - 'p': 普通参数模式，将 Value 作为 float 类型写入 data[4:7]
 *                   - 'j': 模式设置，将 Value 作为 uint8 类型写入 data[4]
 * 
 * @note 发送命令后会调用 receive_status_frame() 接收电机的反馈
 * 
 * @see Set_parameter 宏定义为 'p'，用于设置普通参数
 * @see Set_mode 宏定义为 'j'，用于设置控制模式
 * 
 * 数据帧格式：
 * - CAN ID: [通信类型(0x12)][主机ID][电机ID]
 * - data[0:1]: 参数索引地址（小端序）
 * - data[2:3]: 保留字节（0x00）
 * - data[4:7]: 参数值（根据 Value_mode 决定格式）
 */
void RobStrideMotor::Set_RobStrite_Motor_parameter(uint16_t Index, float Value, char Value_mode)
{
    struct can_frame frame{};

    frame.can_id = Communication_Type_SetSingleParameter << 24 | motor_id << 8 | master_id;
    frame.can_id |= CAN_EFF_FLAG; // 扩展帧
    frame.can_dlc = 0x08;

    frame.data[0] = Index;
    frame.data[1] = Index >> 8;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;

    if (Value_mode == 'p')
    {
        memcpy(&frame.data[4], &Value, 4);
    }
    else if (Value_mode == 'j')
    {
        // Motor_Set_All.set_motor_mode = int(Value);
        frame.data[4] = (uint8_t)Value;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
    }
    int n = write(socket_fd, &frame, sizeof(frame));
    if (n != sizeof(frame))
    {
        perror("set mode failed");
    }
    else
    {
        std::cout << "[✓] Motor set-mode command sent." << std::endl;
    }
    receive_status_frame();
}

/**
 * @brief 发送电机使能指令（通信类型 0x03）
 *
 * 构造并发送扩展帧（`CAN_EFF_FLAG`），CAN ID 由通信类型、`master_id` 与 `motor_id`
 * 组合而成，`DLC=8` 且 `data[0..7]` 全为 0。发送后调用 `receive_status_frame()` 接收一次
 * 电机反馈并解析，更新当前电机状态。
 *
 * 帧格式
 * - CAN ID: [Communication_Type_MotorEnable][master_id][motor_id]（扩展帧）
 * - DLC: 8
 * - Data: 00 00 00 00 00 00 00 00
 *
 * 前置条件
 * - 已调用 `init_socket()` 完成 CAN 套接字创建、绑定与过滤器配置
 *
 * 行为说明
 * - 写帧失败时通过 `perror` 打印错误，但仍会尝试接收一次反馈
 * - 成功接收反馈后，成员变量 `position_ / velocity_ / torque_ / temperature_` 被更新
 *
 * @return std::tuple<float,float,float,float>
 *         分别为当前位置(rad)、速度(rad/s)、转矩(N·m)、温度(℃) 的最新值
 *
 * @throw std::runtime_error 当 `receive_status_frame()` 未收到有效帧、数据不足或通信类型无效
 *
 * @see init_socket()
 * @see receive_status_frame()
 */
std::tuple<float, float, float, float> RobStrideMotor::enable_motor()
{
    struct can_frame frame{};
    frame.can_id = (Communication_Type_MotorEnable << 24) | (motor_id << 8) | master_id;
    frame.can_id |= CAN_EFF_FLAG; // 扩展帧
    frame.can_dlc = 8;
    memset(frame.data, 0, 8);

    int n = write(socket_fd, &frame, sizeof(frame));
    if (n != sizeof(frame))
    {
        perror("enable_motor failed");
    }
    else
    {
        std::cout << "[✓] Motor enable command sent." << std::endl;
    }
    receive_status_frame();
    
    return std::make_tuple(position_, velocity_, torque_, temperature_);
}

/**
 * @brief 将浮点值按区间线性量化为无符号整型
 *
 * 先将输入 `x` 饱和到区间 `[x_min, x_max]`，再按线性比例映射到
 * `[0, 2^bits - 1]`，用于将物理量（角度/速度/电流等）编码为定宽位域。
 *
 * 映射关系
 * - y = (x - x_min) / (x_max - x_min) * (2^bits - 1)
 * - 返回值为对 y 的截断转换（不四舍五入）
 *
 * @param x      待量化的浮点值
 * @param x_min  量化下界（含）
 * @param x_max  量化上界（含）
 * @param bits   目标位宽（建议 1~16）
 * @return uint16_t 量化后的无符号整数，范围 [0, 2^bits - 1]
 *
 * @note
 * - 输入会被饱和到 `[x_min, x_max]` 后再量化
 * - 仅做截断而非四舍五入；如需四舍五入需在外部处理
 * - 要求 `x_max > x_min` 且 `bits >= 1`；否则可能产生除零或溢出
 *
 * @example
 * // 将角度 [-4π, 4π] 量化为 16 位：
 * // uint16_t u = float_to_uint(theta, -4*M_PI, 4*M_PI, 16);
 */
uint16_t RobStrideMotor::float_to_uint(float x, float x_min, float x_max, int bits)
{
    if (x < x_min)
        x = x_min;
    if (x > x_max)
        x = x_max;
    float span = x_max - x_min;
    float offset = x - x_min;
    return static_cast<uint16_t>((offset * ((1 << bits) - 1)) / span);
}

/**
 * @brief 发送运动控制指令（力矩+位置+速度+KP+KD，通信类型 0x12）
 *
 * 按执行器能力上限将输入的力矩/位置/速度/KP/KD 饱和并量化为 16 位后发送。
 * 使用扩展帧：力矩量化值嵌入 CAN ID 的 [bit8..bit23]，其余四个量置于数据区（大端）。
 *
 * 帧格式
 * - CAN ID: [Communication_Type_MotionControl(0x12)][torque_u16][motor_id]
 * - DLC: 8
 * - Data（大端）:
 *   - data[0..1]: position_u16（rad）
 *   - data[2..3]: velocity_u16（rad/s）
 *   - data[4..5]: kp_u16
 *   - data[6..7]: kd_u16
 *
 * 模式处理
 * - 若 `drw.run_mode.data != 0` 且 `pattern == 2`，先发送停机，再将 0x7005 置为 `move_control_mode` 并读取确认
 *
 * @param torque         期望力矩（N·m），量化区间 ±torque_max（嵌入 CAN ID）
 * @param position_rad   期望位置（rad），量化区间 ±position_max
 * @param velocity_rad_s 期望速度（rad/s），量化区间 ±velocity_max
 * @param kp             位置环 KP，量化区间 [0, kp_max]
 * @param kd             速度环 KD，量化区间 [0, kd_max]
 *
 * @return std::tuple<float,float,float,float>
 *         返回最新反馈的 {位置(rad), 速度(rad/s), 转矩(N·m), 温度(℃)}
 *
 * @throw std::runtime_error 当接收反馈无效或通信类型错误（来自 `receive_status_frame()`）
 *
 * @note
 * - 数据区按大端写入；量化前会进行饱和
 * - 写帧失败会 `perror` 打印，但仍会尝试接收一次反馈
 */
std::tuple<float, float, float, float> RobStrideMotor::send_motion_command(float torque,
                                                                           float position_rad,
                                                                           float velocity_rad_s,
                                                                           float kp,
                                                                           float kd)
{
    if(drw.run_mode.data != 0 && pattern == 2)
    {
        Disenable_Motor(0);        
        usleep(1000);

        Set_RobStrite_Motor_parameter(0X7005, move_control_mode, Set_mode);
        usleep(1000);

        Get_RobStrite_Motor_parameter(0x7005);
        usleep(1000);
    }
    struct can_frame frame{};
    frame.can_id = (Communication_Type_MotionControl << 24) | (float_to_uint(torque, -ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type)).torque, ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type)).torque, 16) << 8) | master_id;
    frame.can_id |= CAN_EFF_FLAG; // 扩展帧
    // frame.can_id = 0x1200fd01;
    frame.can_dlc = 8;

    uint16_t pos = float_to_uint(position_rad, -ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type)).position, ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type)).position, 16);
    uint16_t vel = float_to_uint(velocity_rad_s, -ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type)).velocity, ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type)).velocity, 16);
    uint16_t kp_u = float_to_uint(kp, 0.0f, ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type)).kp, 16);
    uint16_t kd_u = float_to_uint(kd, 0.0f, ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type)).kd, 16);

    frame.data[0] = (pos >> 8);
    frame.data[1] = pos;
    frame.data[2] = (vel >> 8);
    frame.data[3] = vel;
    frame.data[4] = (kp_u >> 8);
    frame.data[5] = kp_u;
    frame.data[6] = (kd_u >> 8);
    frame.data[7] = kd_u;
    // 05 70 00 00 07 01 82 F9

    int n = write(socket_fd, &frame, sizeof(frame));

    if (n != sizeof(frame))
    {
        perror("send_motion_command failed");
    }
    receive_status_frame();
    return std::make_tuple(position_, velocity_, torque_, temperature_);
}

/**
 * @brief 发送速度模式指令（设置速度控制模式并下发目标速度）
 *
 * 当当前运行模式非速度模式时（`drw.run_mode.data != 2` 且 `pattern == 2`）：
 * 1) 发送停机；
 * 2) 将 0x7005 写为 `Speed_control_mode` 并读取确认；
 * 3) 重新使能电机；
 * 4) 写入相关参数（如 0x7018、0x7026）。
 * 随后将目标速度写入 0x700A。各次参数写入均会在内部调用 `receive_status_frame()` 并解析反馈。
 *
 * 参数寄存器
 * - 0x7005: 运行模式（设置为速度控制模式）
 * - 0x700A: 速度指令（rad/s）
 * - 0x7018: 速度模式相关参数（此处固定写入 27.0f）
 * - 0x7026: 加速度参数（使用 `Motor_Set_All.set_acc`）
 *
 * 时序说明
 * - 调整模式及参数期间使用若干 `usleep(1000)` 进行毫秒级间隔
 *
 * @param velocity_rad_s 期望速度（rad/s）
 * @return std::tuple<float,float,float,float>
 *         返回最新反馈的 {位置(rad), 速度(rad/s), 转矩(N·m), 温度(℃)}
 *
 * @throw std::runtime_error 当内部的 `receive_status_frame()` 未收到有效帧或通信类型无效
 *
 * @note 需要已完成 `init_socket()`；写帧失败将通过 `perror` 打印错误信息
 * @see Set_RobStrite_Motor_parameter, Get_RobStrite_Motor_parameter, enable_motor, Disenable_Motor
 */
std::tuple<float, float, float, float> RobStrideMotor::send_velocity_mode_command(float velocity_rad_s)
{
    if(drw.run_mode.data != 2 && pattern == 2)
    {
        Disenable_Motor(0);
        std::cout << "disable motor " << std::endl;
        usleep(1000);
        Set_RobStrite_Motor_parameter(0X7005, Speed_control_mode, Set_mode);
        usleep(1000);
        Get_RobStrite_Motor_parameter(0x7005);
        usleep(1000);
        enable_motor();
        Set_RobStrite_Motor_parameter(0X7018, 27.0f, Set_parameter);
        usleep(1000);
        Set_RobStrite_Motor_parameter(0X7026, Motor_Set_All.set_acc,   Set_parameter);
        usleep(1000);
    }
    std::cout << "excute vel_mode" << std::endl;
    Set_RobStrite_Motor_parameter(0X700A, velocity_rad_s, Set_parameter);
    std::cout << "finish" << std::endl;
    return std::make_tuple(position_, velocity_, torque_, temperature_);
}

/**
 * @brief 读取电机上电/启动后的初始位置（阻塞，最长约 10s）
 *
 * 循环监听 CAN 扩展帧，仅当帧类型为 0x02 且 `mid==0x01`、`eid==0xFD` 时，
 * 取 `data[0..1]`（大端）作为位置原始值，并按区间 [-4π, 4π]、16 位分辨率
 * 通过 `uint_to_float` 转换为弧度后返回。若 10 秒内未命中有效帧则返回 0.0。
 *
 * 解析要点
 * - 仅处理带 `CAN_EFF_FLAG` 的扩展帧
 * - `type = (canid >> 24) & 0xFF`，`mid = (canid >> 8) & 0xFF`，`eid = canid & 0xFF`
 * - 位置字段：`p_uint = (data[0] << 8) | data[1]`（大端）
 * - 物理量换算：`pos = uint_to_float(p_uint, -4*M_PI, 4*M_PI, 16)`
 *
 * 前置条件
 * - 已调用 `init_socket()` 完成 CAN 套接字与过滤器配置
 *
 * @return float 初始位置（rad）。超时未获取到有效帧时返回 0.0。
 *
 * @note
 * - 本实现对 `mid`/`eid` 使用了固定值（0x01/0xFD）；若设备 ID 不同需相应调整判定。
 * - 函数包含调试打印，如需静默请移除相关 `printf`/`std::cout`。
 */
float RobStrideMotor::read_initial_position()
{
    struct can_frame frame{};
    auto start = std::chrono::steady_clock::now();
    while (true)
    {
        // float neutral_pos = 2.0f;
        // send_motion_command(neutral_pos, 0.0f, 0.0f, 0.0f);

        ssize_t nbytes = read(socket_fd, &frame, sizeof(frame));
        if (nbytes > 0 && (frame.can_id & CAN_EFF_FLAG))
        {
            uint32_t canid = frame.can_id & CAN_EFF_MASK;
            uint8_t type = (canid >> 24) & 0xFF;
            uint8_t mid = (canid >> 8) & 0xFF;
            uint8_t eid = canid & 0xFF;

            // please switch print output;
            printf("type: 0x%02X\n", type);
            printf("mid:  0x%02X\n", mid);
            printf("eid:  0x%02X\n", eid);

            if (type == 0x02 && mid == 0x01 && eid == 0xFD)
            {
                uint16_t p_uint = (frame.data[0] << 8) | frame.data[1];
                float pos = uint_to_float(p_uint, -4 * M_PI, 4 * M_PI, 16);
                std::cout << "[✓] Initial position read: " << pos << " rad" << std::endl;
                return pos;
            }
        }
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > 10000)
        {
            std::cerr << "[!] Timeout waiting for motor feedback." << std::endl;
            return 0.0f;
        }
    }
}

/**
 * @brief 读取电机的单个参数值（按索引发起查询）
 *
 * 构造并发送扩展帧，请求读取索引 `Index` 对应的参数。发送后立即调用
 * `receive_status_frame()` 等待并解析设备反馈；当反馈通信类型为 17（0x11）
 * 时，会将读到的参数值映射到 `drw` 或 `params` 等成员中。
 *
 * 帧格式
 * - CAN ID: [Communication_Type_GetSingleParameter][master_id][motor_id]（扩展帧）
 * - DLC: 8
 * - Data（小端）:
 *   - data[0..1]: 参数索引 `Index`（低字节在前）
 *   - data[2..7]: 0x00 填充
 *
 * @param Index 参数索引地址（如 0x7005 运行模式等）
 *
 * @note
 * - 需已完成 `init_socket()`；写帧失败会通过 `perror` 打印错误
 * - 实际参数值在 `receive_status_frame()` 中解析并更新到成员变量
 *
 * @throw std::runtime_error 当未收到有效反馈、数据不足或通信类型无效（由 `receive_status_frame()` 抛出）
 *
 * @see receive_status_frame()
 */
void RobStrideMotor::Get_RobStrite_Motor_parameter(uint16_t Index)
{
    struct can_frame frame{};
    frame.can_id = (Communication_Type_GetSingleParameter << 24) | (motor_id << 8) | master_id;
    frame.can_id |= CAN_EFF_FLAG; // 扩展帧
    frame.can_dlc = 8;

    frame.data[0] = Index;
    frame.data[1] = Index>>8;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    int n = write(socket_fd, &frame, sizeof(frame));

    if (n != sizeof(frame))
    {
        perror("get_motor_parameter failed");
    }
    std::cout << "excute Get_motor_params" << std::endl;
    receive_status_frame();

}

/**
 * @brief 位置点到点控制（PosPP）：设置速度、加速度与目标角度
 *
 * 当当前运行模式非 PosPP 模式时（`drw.run_mode.data != 1` 且 `pattern == 2`）：
 * 1) 发送停机；
 * 2) 将 0x7005 写为 `PosPP_control_mode` 并读取确认；
 * 3) 重新使能电机。
 * 随后依次写入 0x7025（速度）、0x7026（加速度）、0x7016（角度）。每次写入内部都会调用
 * `receive_status_frame()` 接收并解析反馈，更新当前状态。
 *
 * 参数寄存器
 * - 0x7005: 运行模式（设置为 PosPP 控制模式）
 * - 0x7025: 速度指令（通常为 rad/s，依固件定义）
 * - 0x7026: 加速度指令（通常为 rad/s^2，依固件定义）
 * - 0x7016: 位置指令（通常为 rad，依固件定义）
 *
 * 时序说明
 * - 模式切换与参数下发之间使用 `usleep(1000)` 做毫秒级间隔
 *
 * @param Speed         目标速度（rad/s，具体以固件定义为准）
 * @param Acceleration  目标加速度（rad/s^2，具体以固件定义为准）
 * @param Angle         目标角度（rad，具体以固件定义为准）
 * @return std::tuple<float,float,float,float>
 *         最新反馈的 {位置(rad), 速度(rad/s), 转矩(N·m), 温度(℃)}
 *
 * @throw std::runtime_error 当内部 `receive_status_frame()` 未收到有效帧或通信类型无效
 *
 * @note
 * - 需先完成 `init_socket()`；写帧失败会通过 `perror` 打印错误
 * - 本函数逐项下发目标值，但不阻塞等待目标角度达到；完成度需由上层逻辑或反馈判断
 * @see Set_RobStrite_Motor_parameter, Get_RobStrite_Motor_parameter, enable_motor, Disenable_Motor
 */
std::tuple<float, float, float, float> RobStrideMotor::RobStrite_Motor_PosPP_control(float Speed, float Acceleration, float Angle)
{
    if(drw.run_mode.data != 1 && pattern == 2)
    {
        Disenable_Motor(0);
        usleep(1000);
        Set_RobStrite_Motor_parameter(0X7005, PosPP_control_mode, Set_mode);
        usleep(1000);
        Get_RobStrite_Motor_parameter(0x7005);
        usleep(1000);
        enable_motor();
        usleep(1000);
    }

	Motor_Set_All.set_speed = Speed;
	Motor_Set_All.set_acc   = Acceleration;
	Motor_Set_All.set_angle = Angle;

	Set_RobStrite_Motor_parameter(0X7025, Motor_Set_All.set_speed, Set_parameter);
    usleep(1000);

	Set_RobStrite_Motor_parameter(0X7026, Motor_Set_All.set_acc,   Set_parameter);
    usleep(1000);

	Set_RobStrite_Motor_parameter(0X7016, Motor_Set_All.set_angle, Set_parameter);
    usleep(1000);

    return std::make_tuple(position_, velocity_, torque_, temperature_);
}

/**
 * @brief 电流控制模式（FOC）：下发 Iq/Id 指令
 *
 * 当当前运行模式非电流模式时（`drw.run_mode.data != 3` 且 `pattern == 2`）：
 * 1) 发送停机；
 * 2) 将 0x7005 写为 `Elect_control_mode` 并读取确认；
 * 3) 重新使能电机。
 * 随后依次写入 Iq/Id 指令寄存器。每次参数写入内部都会调用
 * `receive_status_frame()` 接收并解析反馈，更新当前状态。
 *
 * 指令寄存器
 * - 0x7005: 运行模式（设置为电流控制模式）
 * - 0x7006: Iq 指令（A），按 [SCIQ_MIN, SC_MAX] 饱和并量化为 16 位后写入
 * - 0x7007: Id 指令（A），直接以浮点值写入
 *
 * 时序说明
 * - 模式切换与参数下发之间使用 `usleep(1000)` 做毫秒级间隔
 *
 * @param IqCommand q 轴电流指令（A）
 * @param IdCommand d 轴电流指令（A）
 * @return std::tuple<float,float,float,float>
 *         最新反馈的 {位置(rad), 速度(rad/s), 转矩(N·m), 温度(℃)}
 *
 * @throw std::runtime_error 当内部 `receive_status_frame()` 未收到有效帧或通信类型无效
 *
 * @note
 * - 需已完成 `init_socket()`；写帧失败会通过 `perror` 打印错误
 * - Iq 会先进行区间饱和与 16 位量化；Id 直接写入
 * @see Set_RobStrite_Motor_parameter, Get_RobStrite_Motor_parameter, enable_motor, Disenable_Motor
 */
std::tuple<float, float, float, float> RobStrideMotor::RobStrite_Motor_Current_control(float IqCommand, float IdCommand) 
{
    if(drw.run_mode.data != 3)
    {
        Disenable_Motor(0);
        usleep(1000);
        Set_RobStrite_Motor_parameter(0X7005, Elect_control_mode, Set_mode);
        usleep(1000);
        Get_RobStrite_Motor_parameter(0x7005);
        usleep(1000);
        enable_motor();
        usleep(1000);
    }

    // Store the target values
    Motor_Set_All.set_iq = IqCommand;
    Motor_Set_All.set_id = IdCommand;

    Motor_Set_All.set_iq = float_to_uint(Motor_Set_All.set_iq, SCIQ_MIN,SC_MAX, 16);
    Set_RobStrite_Motor_parameter(0X7006, Motor_Set_All.set_iq, Set_parameter);
    usleep(1000);

    Set_RobStrite_Motor_parameter(0X7007, Motor_Set_All.set_id, Set_parameter);
    usleep(1000);

    return std::make_tuple(position_, velocity_, torque_, temperature_);

}

/**
 * @brief 切换至“零位设置”运行模式
 *
 * 通过将寄存器 0x7005（运行模式）写为 `Set_Zero_mode` 来进入零位设置模式，
 * 内部调用 `Set_RobStrite_Motor_parameter()` 发送参数写入并在其内部解析一次反馈。
 * 本函数仅切换模式，不直接执行零位写入动作。
 *
 * @note
 * - 实际的零位写入请使用 `Set_ZeroPos()`（发送对应的置零命令帧）
 * - 需已完成 `init_socket()`；写帧失败会通过 `perror` 打印（由内部函数处理）
 *
 * @throw std::runtime_error 若内部的 `receive_status_frame()` 未收到有效反馈或通信类型无效
 *
 * @see Set_RobStrite_Motor_parameter
 * @see Set_ZeroPos
 */
void RobStrideMotor::RobStrite_Motor_Set_Zero_control()
{
	Set_RobStrite_Motor_parameter(0X7005, Set_Zero_mode, Set_mode);					//设置电机模式
}

/**
 * @brief 停止电机（可选清除故障），发送 MotorStop 指令
 *
 * 构造并发送扩展帧以停止电机运行。`data[0]` 作为清错开关，其余字节填 0。
 * 发送后调用 `receive_status_frame()` 接收并解析一次反馈，更新当前状态。
 *
 * 帧格式
 * - CAN ID: [Communication_Type_MotorStop][master_id][motor_id]（扩展帧）
 * - DLC: 8
 * - Data: [clear_error, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 *
 * @param clear_error 是否清除故障标志（1 清除，0 不清除）
 *
 * @note 需已完成 `init_socket()`；写帧失败会通过 `perror` 打印错误信息
 * @throw std::runtime_error 当 `receive_status_frame()` 未收到有效帧或通信类型无效
 *
 * @see enable_motor
 * @see receive_status_frame
 */
void RobStrideMotor::Disenable_Motor(uint8_t clear_error)
{
    struct can_frame frame{};
    frame.can_id = (Communication_Type_MotorStop << 24) | (motor_id << 8) | master_id;
    frame.can_id |= CAN_EFF_FLAG; // 扩展帧
    frame.can_dlc = 8;
    memset(frame.data, 0, 8);

    frame.data[0] = clear_error;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    int n = write(socket_fd, &frame, sizeof(frame));

    if (n != sizeof(frame))
    {
        perror("disable_motor failed");
    }
    else
    {
        std::cout << "[✓] Motor disable command sent." << std::endl;
    }
    receive_status_frame();
}

/**
 * @brief 设置电机的 CAN 从站 ID（扩展帧）
 *
 * 先发送停机指令（`Disenable_Motor(0)`），随后下发“修改 CAN ID”命令。
 * 新的从站 ID 放入 CAN ID 的 [bit16..bit23] 字段，数据区全 0。本函数不读取确认帧，
 * 仅负责发送命令，设备后续将以新 ID 响应。
 *
 * 帧格式
 * - CAN ID: [Communication_Type_Can_ID][Set_CAN_ID][master_id][motor_id]（扩展帧）
 * - DLC: 8
 * - Data: 00 00 00 00 00 00 00 00
 *
 * 前置条件
 * - 已完成 `init_socket()`；函数内部会先停机以确保安全修改
 *
 * 使用注意
 * - 发送成功后，设备通信 ID 切换为 `Set_CAN_ID`；当前对象的 `motor_id` 与套接字过滤器并不会自动更新，
 *   调用方应同步更新成员并重新配置过滤器（必要时重新调用 `init_socket()`），否则可能收不到新 ID 的反馈
 * - 是否需要掉电重启或保存到非易失存储，取决于设备固件实现
 *
 * @param Set_CAN_ID 目标 CAN 从站 ID（0~255）
 *
 * @see Disenable_Motor
 * @see init_socket
 */
void RobStrideMotor::Set_CAN_ID(uint8_t Set_CAN_ID)
{
	Disenable_Motor(0);

    struct can_frame frame{};
    frame.can_id = (Communication_Type_Can_ID<<24) | (Set_CAN_ID<<16) | (motor_id << 8) | master_id;
    frame.can_id |= CAN_EFF_FLAG; // 扩展帧
    frame.can_dlc = 8;
    memset(frame.data, 0, 8);

    frame.data[0] = 0x00;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    int n = write(socket_fd, &frame, sizeof(frame));

    if (n != sizeof(frame))
    {
        perror("Set_ZeroPos failed");
    }
    else
    {
        std::cout << "[✓] Motor Set_ZeroPos command sent." << std::endl;
    }
}

/**
 * @brief 位置模式 CSP（Cyclic Synchronous Position）：周期同步位置控制
 *
 * 当当前运行模式非 PosCSP 模式时（`drw.run_mode.data != 5` 且 `pattern == 2`）：
 * 1) 发送停机；
 * 2) 将 0x7005 写为 `PosCSP_control_mode` 并读取确认；
 * 3) 重新使能电机。
 * 随后将速度量化为 16 位并写入 0x7017，再将目标角度写入 0x7016。
 * 每次参数写入内部都会调用 `receive_status_frame()` 接收并解析反馈，更新当前状态。
 *
 * CSP 模式特点
 * - 支持实时跟随：以固定周期发送位置指令，电机连续跟踪目标位置
 * - 速度作为约束：0x7017 设定运动速度上限，电机在此范围内规划轨迹
 * - 适用于需要轨迹插补或外部路径规划的应用场景
 *
 * 参数寄存器
 * - 0x7005: 运行模式（设置为 PosCSP 控制模式）
 * - 0x7017: 速度约束（rad/s，量化为 16 位后写入）
 * - 0x7016: 目标位置（rad，直接以浮点值写入）
 *
 * 时序说明
 * - 模式切换与参数下发之间使用 `usleep(1000)` 做毫秒级间隔
 * - 速度需按 [-velocity_max, +velocity_max] 量化为 16 位后再写入
 *
 * @param Speed 运动速度约束（rad/s），量化区间由 `actuator_type` 对应的能力上限决定
 * @param Angle 目标位置（rad）
 * @return std::tuple<float,float,float,float>
 *         最新反馈的 {位置(rad), 速度(rad/s), 转矩(N·m), 温度(℃)}
 *
 * @throw std::runtime_error 当内部 `receive_status_frame()` 未收到有效帧或通信类型无效
 *
 * @note
 * - 需已完成 `init_socket()`；写帧失败会通过 `perror` 打印错误
 * - CSP 模式适合高频周期调用（如 1kHz），每次更新目标位置以实现轨迹跟踪
 * - 速度参数作为约束而非指令，实际速度由位置差和控制周期决定
 *
 * @see Set_RobStrite_Motor_parameter, Get_RobStrite_Motor_parameter, enable_motor, Disenable_Motor
 * @see RobStrite_Motor_PosPP_control (点到点位置模式，用于单次定位)
 */
std::tuple<float, float, float, float> RobStrideMotor::RobStrite_Motor_PosCSP_control(float Speed, float Angle)
{
	Motor_Set_All.set_speed = Speed;
	Motor_Set_All.set_angle = Angle;
	if (drw.run_mode.data != 5 && pattern == 2)
	{
        Disenable_Motor(0);
        usleep(1000);
		Set_RobStrite_Motor_parameter(0X7005, PosCSP_control_mode, Set_mode);		//设置电机模式
        usleep(1000);

		Get_RobStrite_Motor_parameter(0x7005);
        usleep(1000);

        enable_motor();
        usleep(1000);

		Motor_Set_All.set_motor_mode = PosCSP_control_mode;
	}
	Motor_Set_All.set_speed = float_to_uint(Motor_Set_All.set_speed, -ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type)).velocity, ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type)).velocity, 16);
	Set_RobStrite_Motor_parameter(0X7017, Motor_Set_All.set_speed, Set_parameter);
    usleep(1000);

	Set_RobStrite_Motor_parameter(0X7016, Motor_Set_All.set_angle, Set_parameter);
    usleep(1000);

    return std::make_tuple(position_, velocity_, torque_, temperature_);
}

/**
 * @brief 将当前位置设置为电机零点（原点标定）
 *
 * 该方法将电机当前所在位置标定为新的零点位置。执行流程：
 * 1) 发送停机指令以确保安全；
 * 2) 若当前模式非模式 4，则切换到速度控制模式并读取确认；
 * 3) 发送 `Communication_Type_SetPosZero` 帧，`data[0]=1` 表示执行零点设置；
 * 4) 重新使能电机。
 *
 * 帧格式
 * - CAN ID: [Communication_Type_SetPosZero][master_id][motor_id]（扩展帧）
 * - DLC: 8
 * - Data: [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 *
 * 使用场景
 * - 机械臂或关节在初次安装后的原点标定
 * - 运行过程中需要重新定义零点位置
 * - 碰撞或失步后的位置校准
 *
 * 前置条件
 * - 已完成 `init_socket()`
 * - 电机应处于期望的零点物理位置
 *
 * 注意事项
 * - 零点设置后，电机的位置编码器将以当前位置为 0 rad 重新计数
 * - 零点信息是否持久化（掉电保存）取决于固件实现
 * - 建议在静止状态下执行零点标定，避免运动中标定导致位置偏差
 *
 * @note
 * - 执行前会先停机（`Disenable_Motor(0)`），执行后自动重新使能
 * - 若当前非模式 4，会先切换到速度控制模式（可能是固件要求的前置模式）
 * - 写帧失败会通过 `perror` 打印 "Set_ZeroPos failed"
 *
 * @see Disenable_Motor
 * @see enable_motor
 * @see Set_RobStrite_Motor_parameter
 * @see RobStride_Motor_Set_Zero_control (仅切换到零位设置模式，不执行实际标定)
 */
void RobStrideMotor::Set_ZeroPos()
{
	Disenable_Motor(0);

    if(drw.run_mode.data != 4)
    {
        Set_RobStrite_Motor_parameter(0X7005, Speed_control_mode, Set_mode);
        usleep(1000);

        Get_RobStrite_Motor_parameter(0x7005);
        usleep(1000);

    }

    struct can_frame frame{};
    frame.can_id = (Communication_Type_SetPosZero << 24) | (motor_id << 8) | master_id;
    frame.can_id |= CAN_EFF_FLAG; // 扩展帧
    frame.can_dlc = 8;
    memset(frame.data, 0, 8);

    frame.data[0] = 1;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    int n = write(socket_fd, &frame, sizeof(frame));

    if (n != sizeof(frame))
    {
        perror("Set_ZeroPos failed");
    }
    else
    {
        std::cout << "[✓] Motor Set_ZeroPos command sent." << std::endl;
    }

	enable_motor();
}
