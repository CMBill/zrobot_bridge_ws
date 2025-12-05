//
// Created by bill on 2025 12 05.
//

#ifndef RS_MOTOR_ROS2_ROBSTRIDEMOTORRECEIVE_H
#define RS_MOTOR_ROS2_ROBSTRIDEMOTORRECEIVE_H

#include <iostream>
#include <cstring>
#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <vector>
#include <optional>

using ReceiveResult = std::optional<std::tuple<uint8_t, uint16_t, uint8_t, std::vector<uint8_t>>>;

class RobStrideMotorReceiver
{
public:
    void recieve_can_data();

private:

    std::string iface;
    int socket_fd_rx = -1;

    void init_receive_socket();

};


#endif //RS_MOTOR_ROS2_ROBSTRIDEMOTORRECEIVE_H