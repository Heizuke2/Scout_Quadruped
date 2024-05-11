#ifndef CAN_COMM_H
#define CAN_COMM_H

#include "../MathOpsLib/math_ops.h"
#include <cstring>
#include <linux/can.h>
#include <sys/time.h>
#include <iostream>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <vector>
#include <typeinfo>

// Limits
#define P_MIN -12.5f  // -4*pi              #in radians
#define P_MAX 12.5f   // 4*pi
#define V_MIN -65.0f
#define V_MAX 65.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f        //originally -18.0
#define T_MAX 10.0f         //originally 18.0

struct MotorStatusStruct {
    std::string name;
    int motor_id;
    float p_des, v_des, kp, kd, t_ff;
};

void open_port(std::string interf_name, int bitrate);
void close_port(std::string can_id);
void pack_msg(can_frame& frame, float pos, float vel, float KP, float KD, float T_FF);
void unpack_msg(std::vector<int> data, MotorStatusStruct& motor);
void enter_motor_mode(const char* interface_name, int motor_id, MotorStatusStruct& motor);
void exit_motor_mode(const char* interface_name, int motor_id, MotorStatusStruct& motor);
void zero_position_sensor(const char* interface_name, int motor_id, MotorStatusStruct& motor);
void write_can_frame(const char* interface_name, int motor_id, float pos, float vel, float KP, float KD, float FF, MotorStatusStruct& motor);

#endif // CAN_COMM_H
