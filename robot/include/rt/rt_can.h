/*!
 * @file rt_can.h
 * @brief CAN communication to CANable
 */

#ifndef _rt_can
#define _rt_can

#include "Actuator.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "actuator_command_t.hpp"
#include "actuator_response_t.hpp"
#include <iostream>
#include <can_data_t.hpp>
#include <can_command_t.hpp>


#define m1  0x01
#define m2  0x02
#define m3  0x03
#define m4  0x04
#define m5  0x05
#define m6  0x06
#define m7  0x07
#define m8  0x08
#define m9  0x09
#define m10 0x0A
#define m11 0x0B
#define m12 0x0C

/*!
 * Data from CAN interface
 */
struct CANData {
  float q_abad[4];
  float q_hip[4];
  float q_knee[4];
  float qd_abad[4];
  float qd_hip[4];
  float qd_knee[4];
};

/*!
 * Command to can interface
 */
struct CANCommand {
  float q_des_abad[4];
  float q_des_hip[4];
  float q_des_knee[4];

  float qd_des_abad[4];
  float qd_des_hip[4];
  float qd_des_knee[4];

  float kp_abad[4];
  float kp_hip[4];
  float kp_knee[4];

  float kd_abad[4];
  float kd_hip[4];
  float kd_knee[4];

  float tau_abad_ff[4];
  float tau_hip_ff[4];
  float tau_knee_ff[4];
};

class CAN {
    public:
    CAN():
        FR("can1"), FL("can2"), RR("can3"), RL("can0") {}
        
    void init_can();
    void can_send_receive(CANCommand* can_command, CANData* can_response);
    void check_safety();
    void stop_can();
    CANCommand* get_can_command();
    CANData *get_can_data();

    private:
    Actuator FR, FL, RR, RL;

    // CAN(): FR("can2") {}
    // Actuator FR;
    // Actuator FR("can2");
    // FL("can1"), RR("can3"), RL("can0");

};





#endif