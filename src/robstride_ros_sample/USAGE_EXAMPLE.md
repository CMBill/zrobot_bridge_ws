# RobStride 电机控制使用示例

## 简介

此示例展示如何在 ROS 节点中使用 `RobStrideMotor` 和 `RobStrideMotorReceiver` 来控制多个电机,并自动接收反馈。

## 主要特性

1. **自动反馈接收**: 每次发送指令后自动等待并获取电机反馈
2. **非阻塞设计**: 接收器在独立线程中运行,不阻塞主程序
3. **多电机支持**: 多个电机共享一个接收器,通过 `motor_id` 区分
4. **线程安全**: 使用互斥锁保护共享数据
5. **超时保护**: 如果通信不畅,不会无限等待

## 使用方法

### 1. 基本使用示例

```cpp
#include "motor_ros2/RobStrideMotor.h"
#include "motor_ros2/RobStrideMotorReceiver.h"

int main()
{
    // 步骤 1: 创建接收器 (所有电机共享一个接收器)
    auto receiver = std::make_shared<RobStrideMotorReceiver>("can0", 0x00);
    
    // 步骤 2: 启动接收器
    if (!receiver->Start())
    {
        std::cerr << "Failed to start receiver!" << std::endl;
        return -1;
    }

    // 步骤 3: 创建电机对象,传入接收器
    RobStrideMotor motor1(0x01, "can0", 0x00, receiver);
    RobStrideMotor motor2(0x02, "can0", 0x00, receiver);
    RobStrideMotor motor3(0x03, "can0", 0x00, receiver);

    // 步骤 4: 发送指令 (自动获取反馈)
    motor1.Enable_Motor();  // 自动打印反馈信息
    motor2.Enable_Motor();
    motor3.Enable_Motor();

    // 控制电机运动
    motor1.Set_MIT_Dynamic_Param(1.0, 0.5, 10.0, 1.0, 0.0);
    motor2.Set_Motor_Angle(3.14, 5.0);
    motor3.Set_Motor_Velocity(2.0, 10.0);

    // 停止电机
    motor1.Disable_Motor();
    motor2.Disable_Motor();
    motor3.Disable_Motor();

    // 步骤 5: 停止接收器
    receiver->Stop();

    return 0;
}
```

### 2. ROS 节点中使用

```cpp
#include "rclcpp/rclcpp.hpp"
#include "motor_ros2/RobStrideMotor.h"
#include "motor_ros2/RobStrideMotorReceiver.h"

class MotorControlNode : public rclcpp::Node
{
public:
    MotorControlNode() : Node("motor_control_node")
    {
        // 创建接收器
        receiver_ = std::make_shared<RobStrideMotorReceiver>("can0", 0x00);
        if (!receiver_->Start())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to start receiver!");
            return;
        }

        // 创建多个电机
        motor1_ = std::make_unique<RobStrideMotor>(0x01, "can0", 0x00, receiver_);
        motor2_ = std::make_unique<RobStrideMotor>(0x02, "can0", 0x00, receiver_);
        motor3_ = std::make_unique<RobStrideMotor>(0x03, "can0", 0x00, receiver_);

        // 使能电机
        motor1_->Enable_Motor();
        motor2_->Enable_Motor();
        motor3_->Enable_Motor();

        // 创建定时器进行周期性控制
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&MotorControlNode::control_loop, this));
    }

    ~MotorControlNode()
    {
        // 停止电机
        if (motor1_) motor1_->Disable_Motor();
        if (motor2_) motor2_->Disable_Motor();
        if (motor3_) motor3_->Disable_Motor();

        // 停止接收器
        if (receiver_) receiver_->Stop();
    }

private:
    void control_loop()
    {
        // 发送控制指令 (自动获取反馈)
        motor1_->Set_MIT_Dynamic_Param(target_angle_, 0.0, 50.0, 2.0, 0.0);
        motor2_->Set_Motor_Angle(target_angle_, 5.0);
        motor3_->Set_Motor_Velocity(target_velocity_, 10.0);

        // 如果需要手动获取反馈数据
        MotorFeedback feedback;
        if (motor1_->GetLatestFeedback(feedback, 50))
        {
            RCLCPP_INFO(this->get_logger(), 
                "Motor 1 - Pos: %.3f, Vel: %.3f, Torque: %.3f",
                feedback.position, feedback.velocity, feedback.torque);
        }
    }

    std::shared_ptr<RobStrideMotorReceiver> receiver_;
    std::unique_ptr<RobStrideMotor> motor1_;
    std::unique_ptr<RobStrideMotor> motor2_;
    std::unique_ptr<RobStrideMotor> motor3_;
    rclcpp::TimerBase::SharedPtr timer_;

    float target_angle_ = 0.0;
    float target_velocity_ = 1.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControlNode>());
    rclcpp::shutdown();
    return 0;
}
```

### 3. 为已创建的电机对象设置接收器

如果电机对象已经创建,可以后续设置接收器:

```cpp
// 创建电机 (不传入接收器)
RobStrideMotor motor1(0x01, "can0", 0x00);
RobStrideMotor motor2(0x02, "can0", 0x00);

// 稍后创建并设置接收器
auto receiver = std::make_shared<RobStrideMotorReceiver>("can0", 0x00);
receiver->Start();

motor1.SetReceiver(receiver);
motor2.SetReceiver(receiver);

// 现在发送指令会自动获取反馈
motor1.Enable_Motor();
motor2.Enable_Motor();
```

## 工作原理

1. **发送指令**: 调用任何电机控制方法(如 `Enable_Motor()`)
2. **清除旧数据**: 自动清除该电机的旧反馈缓存
3. **发送 CAN 帧**: 通过 CAN 总线发送指令
4. **等待反馈**: 等待接收器接收到对应电机的反馈(默认超时 50ms)
5. **打印信息**: 自动打印反馈数据(位置、速度、扭矩)

## 输出示例

```
[✓] RobStrideMotorReceiver initialized on can0 with master_id: 0x0
[✓] RobStrideMotorReceiver started
[✓] Motor 1 - Pos: 0.125, Vel: 0.050, Torque: 0.230
[✓] Motor 1 enable command sent.
[✓] Motor 2 - Pos: 0.000, Vel: 0.000, Torque: 0.000
[✓] Motor 2 enable command sent.
[✓] Motor 3 - Pos: 1.570, Vel: 1.200, Torque: 5.600
[✓] Motor 3 MIT ctrl params sent.
```

## 注意事项

1. **接收器共享**: 多个电机应该共享同一个接收器实例
2. **线程安全**: 接收器是线程安全的,可以在多个线程中使用电机对象
3. **超时设置**: 如果通信质量差,可以增加超时时间
4. **反馈格式**: 请根据实际电机协议调整 `ParseFeedback` 方法中的数据解析逻辑
5. **非阻塞**: 如果接收器未启动或未设置,发送操作仍会成功,只是不会等待反馈

## 自定义超时时间

可以通过 `GetLatestFeedback` 方法自定义超时时间:

```cpp
MotorFeedback feedback;
// 等待 100ms
if (motor1.GetLatestFeedback(feedback, 100))
{
    std::cout << "Position: " << feedback.position << std::endl;
}
```

## 错误处理

```cpp
// 发送指令
motor1.Enable_Motor();

// 检查是否成功获取反馈
MotorFeedback feedback;
if (!motor1.GetLatestFeedback(feedback, 50))
{
    std::cerr << "Failed to get feedback from motor 1" << std::endl;
    // 处理错误...
}
```
