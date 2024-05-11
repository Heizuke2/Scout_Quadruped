#ifndef CAN_COMM_H
#define CAN_COMM_H

/*
#ifndef PF_CAN
#define PF_CAN 29
#endif
*/

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

// struct MotorStatusStruct
// {
//     std::string name;
//     int motor_id;
//     int p_des, v_des, kp,kd,t_ff;
// };


// void open_port(std::string can_id, int bitrate);
// void close_port(std::string can_id);
// void pack_msg(can_frame& frame, float pos, float vel, float KP, float KD, float T_FF);
// void unpack_msg(std::vector<int> data, MotorStatusStruct& motor);
// void enter_motor_mode(const char* interface_name, int id_can, MotorStatusStruct& motor);
// void exit_motor_mode(const char* interface_name, int id_can, MotorStatusStruct& motor);
// void zero_position_sensor(const char* interface_name, int id_can, MotorStatusStruct& motor);
// void write_can_frame(const char* interface_name, int id_can, int pos, int vel, int KP, int KD, int FF, MotorStatusStruct& motor);


// #endif



class MotorController {
public:
    MotorController(const char* interface_name, int motorID);
    void enter_motor_mode();
    void exit_motor_mode();
    void zero_position_sensor();
    void write_can_frame(float pos, float vel, float KP, float KD, float FF);
    float getPDes() const;
    float getVDes() const;
    float getTFF() const;

private:
    const char* interf_name;
    int motor_id;
    float p_des, v_des, kp, kd, t_ff;
    struct sockaddr_can addr;
    struct ifreq ifr;
    socklen_t len;

    void unpack_msg(std::vector<int> data);
    void pack_msg(can_frame& frame, float pos, float vel, float KP, float KD, float T_FF);
};

void open_port(std::string interf_name, int bitrate);
void close_port(std::string can_id);
float radToDeg(float rad);
float degToRad(float deg);
void can_write(int position, int motor_id);

#endif // MOTOR_CONTROLLER_H