/*!
 * @file rt_joystick_interface.h
 * @brief Communication with USB Joystick
 */

#ifndef _rt_joystick_interface
#define _rt_joystick_interface

#include <stdint.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/joystick.h>


// class rc_control_settings {
//   public:
//     double     mode;
//     double     p_des[2]; // (x, y) -1 ~ 1
//     double     height_variation; // -1 ~ 1
//     double     v_des[3]; // -1 ~ 1 * (scale 0.5 ~ 1.5)
//     double     rpy_des[3]; // -1 ~ 1
//     double     omega_des[3]; // -1 ~ 1
//     double     variable[3];
//     double     step_height;
// };

// namespace RC_mode{
//   constexpr int OFF = 0;
//   constexpr int STAND_UP = 1;
//   constexpr int SQUAT_DOWN = 2;
//   constexpr int QP_STAND = 3;
//   constexpr int BACKFLIP_PRE = 4;
//   constexpr int BACKFLIP = 5;
//   constexpr int VISION = 6;
//   constexpr int LOCOMOTION = 11;
//   constexpr int RECOVERY_STAND = 12;

//   // Experiment Mode
//   constexpr int TWO_LEG_STANCE_PRE = 20;
//   constexpr int TWO_LEG_STANCE = 21;
// };


void get_js_control_settings(void* settings);

int receive_data(int fd);

int init_joystick();

void update_joystick();

//void* v_memcpy(void* dest, volatile void* src, size_t n);
#endif