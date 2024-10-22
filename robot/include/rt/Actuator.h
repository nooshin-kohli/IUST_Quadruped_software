#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <linux/can.h>
#include <string>

#include <lcm/lcm-cpp.hpp>

#include "SocketCAN.h"

#include "actuator_response_t.hpp"


class Actuator {
    private:
        SocketCAN* adapter;
        lcm::LCM lcm;
        actuator_response_t DATA;
    public:
        /**
         * @brief response count for each actuator
         * 
         */
        int responseCount;
        /**
         * @brief optional timeout after each command transmit
         * 
         */
	// double command_timeout;
        /**
         * @brief motor responses in each cycle
         * 
         */
        int m_index;
        float **cycle_responses;
        int __data[8] = {0,0,0,0,0,0,0,0};
        float __pose_shift;
        struct can_filter rfilter[1];
        /**
         * @brief Construct a new Actuator object
         * 
         * @param can_index 
         */
        Actuator(std::string can_index);
        /**
         * @brief reception handler callback function
         * 
         */
        void rx_handler(can_frame_t*);
        /**
         * @brief enable motor with requested id 
         * 
         * @param id 
         */
        void enable(int id);
        /**
         * @brief disable motor with requested id 
         * 
         * @param id 
         */
        void disable(int id);
        /**
         * @brief send null command (for testing)
         * 
         * @param id 
         */
        void zero(int id);
        /**
         * @brief pack and transmit pid commmand to motor with requested id
         * 
         * @param id 
         * @param p_d 
         * @param v_d 
         * @param kp 
         * @param kd 
         * @param ff 
         */
        void command(int id,double p_d,double v_d,double kp,double kd,double ff);
        /**
         * @brief unpack data recieved from motors
         * 
         * @param frame 
         * @return float* 
         */
        float * unpack_data(can_frame_t frame);
        /**
         * @brief pack data to appropriate format to send to motors
         * 
         * @param p_d 
         * @param v_d 
         * @param kp 
         * @param kd 
         * @param ff 
         * @return float* 
         */
        float * pack_data(double p_d,double v_d,double kp,double kd,double ff);
        /**
         * @brief convert decimal to binary
         * 
         * @param n 
         * @param binaryNum 
         */
        void static decToBinary(int n, int* binaryNum);
        /**
         * @brief convert binary to decimal
         * 
         * @param binaryNum 
         * @return int 
         */
        int static binaryToDec(int* binaryNum);
};

#endif
