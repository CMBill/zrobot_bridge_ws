//
// Created by bill on 2025 12 05.
//

#include "../include/motor_ros2/RobStrideMotorReceiver.h"

#include <unistd.h>

void RobStrideMotorReceiver::init_receive_socket()
{
    socket_fd_rx = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_rx < 0)
    {
        perror("receive socket");
        exit(1);
    }

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ);
    if (ioctl(socket_fd_rx, SIOCGIFINDEX, &ifr) < 0)
    {
        perror("ioctl");
        exit(1);
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_rx, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
    {
        perror("bind");
        exit(1);
    }
}

void RobStrideMotorReceiver::recieve_can_data()
{
    struct can_frame frame;
    while (true)
    {
        ssize_t nbytes = read(socket_fd_rx, reinterpret_cast<char*>(&frame), sizeof(frame));

    }
}

