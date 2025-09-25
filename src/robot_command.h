#ifndef ROBOT_COMMAND_H
#define ROBOT_COMMAND_H

#include <Arduino.h>

struct RobotCommand {
    uint8_t id;
    int32_t vx_linear;
    int32_t vy_linear;
    int32_t angular_speed;
    int32_t angle;
    bool kick_front;
};

RobotCommand decodeCommand(const char* buffer);

void printCommand(const RobotCommand& cmd);

#endif