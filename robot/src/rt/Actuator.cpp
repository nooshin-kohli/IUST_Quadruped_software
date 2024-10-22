#include <iostream>
#include <ratio>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can/raw.h>
#include <chrono>
#include <thread>
#include <math.h>
#include <ctime>
#include <unistd.h>

#include "rt/Actuator.h"


using namespace std;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::system_clock;

/// Actuator class
Actuator::Actuator(string can_index) {
    // double command_timeout = 10000 ; // micro seconds
    __pose_shift = 0.5;
    m_index = 0;
    cycle_responses = new float*[3];
    for( int i=0; i<3; ++i ) {
        cycle_responses[i] = new float[4];
    }
    responseCount = 0;
    adapter = new SocketCAN(std::bind(&Actuator::rx_handler, this, std::placeholders::_1));
    adapter->open(can_index.c_str());
};

/// callback function to handle recieved data from motors
void Actuator::rx_handler(can_frame_t* frame) {
    cycle_responses[m_index][0] = frame->data[0];

    int p_data = frame->data[1] * 16 * 16 + frame->data[2];
    float p = p_data * (190. / 65535.) - 95.5;
    cycle_responses[m_index][1] = p;

    int b4[8];
    this->decToBinary(frame->data[4], b4);

    int highBin[4] = {b4[0], b4[1], b4[2], b4[3]};
    int lowBin[4] = {b4[4], b4[5], b4[6], b4[7]};
    int d4_high = this->binaryToDec(highBin);

    int d4_low = this->binaryToDec(lowBin);

    int v_data = frame->data[3] * 16 + d4_high;

    float v = (v_data * 90) / 4095. - 45;

    cycle_responses[m_index][2] = v;

    int i_data = d4_low * 16 * 16 + frame->data[5];
    float i = (i_data * 36) / 4095. - 18;
    cycle_responses[m_index][3] = i;

    // pack it and send it to lcm
    DATA.id = cycle_responses[m_index][0];
    DATA.position = cycle_responses[m_index][1];
    DATA.velocity = cycle_responses[m_index][2];
    DATA.current = cycle_responses[m_index][3];
    lcm.publish("RESP", &DATA);
    responseCount++;
    m_index = (m_index+1)%3;
}

/// command to enable motor 
void Actuator::enable(int id) {
    can_frame_t frame;

    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = 255;
    frame.data[1] = 255;
    frame.data[2] = 255;
    frame.data[3] = 255;
    frame.data[4] = 255;
    frame.data[5] = 255;
    frame.data[6] = 255;
    frame.data[7] = 252;
    adapter->transmit(&frame);
}

/// command to disable motor
void Actuator::disable(int id) {
    can_frame_t frame;

    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = 255;
    frame.data[1] = 255;
    frame.data[2] = 255;
    frame.data[3] = 255;
    frame.data[4] = 255;
    frame.data[5] = 255;
    frame.data[6] = 255;
    frame.data[7] = 253;

    adapter->transmit(&frame);
}

/// null command
void Actuator::zero(int id) {
    can_frame_t frame;

    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = 255;
    frame.data[1] = 255;
    frame.data[2] = 255;
    frame.data[3] = 255;
    frame.data[4] = 255;
    frame.data[5] = 255;
    frame.data[6] = 255;
    frame.data[7] = 254;

    adapter->transmit(&frame);
}

/// sending pid commands to motor
void Actuator::command(int id, double p_d, double v_d, double kp, double kd, double ff) {
    can_frame_t frame;

    float *data = this->pack_data(p_d + this->__pose_shift, v_d, kp, kd, ff);

    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = data[0];
    frame.data[1] = data[1];
    frame.data[2] = data[2];
    frame.data[3] = data[3];
    frame.data[4] = data[4];
    frame.data[5] = data[5];
    frame.data[6] = data[6];
    frame.data[7] = data[7];

    adapter->transmit(&frame);
    delete[] data;

    // if needed there can be a delay between each send adjusted by timeout (uncomment below line) 
    // std::this_thread::sleep_for(std::chrono::microseconds(command_timeout));
}

float *Actuator::pack_data(double p_d, double v_d, double kp, double kd, double ff) {
    float *result = new float[8]; // delete allocated memory
    int p_data = static_cast<int>(((p_d + 95.5) * 65535 / 191));
    int v_data = static_cast<int>(((v_d + 45) * 4095 / 90));
    int kp_data = static_cast<int>((kp * 4095 / 500));
    int kd_data = static_cast<int>((kd * 4095 / 5));
    int ff_data = static_cast<int>(((ff + 18) * 4095 / 36));
    result[0] = p_data >> 8;
    result[1] = p_data & 0xFF;
    result[2] = v_data >> 4;
    result[3] = ((v_data & 0xF) << 4) | (kp_data >> 8);
    result[4] = kp_data & 0xFF;
    result[5] = kd_data >> 4;
    result[6] = ((kd_data & 0xF) << 4) | (ff_data >> 8);
    result[7] = ff_data & 0xFF;
    return result;
}

float *Actuator::unpack_data(can_frame_t frame) {
    float *result = new float[4];
    result[0] = frame.data[0];
    int p_data = frame.data[1] * 16 * 16 + frame.data[2];
    float p = p_data * (190. / 65535.) - 95.5;
    result[1] = p;

    int b4[8];
    this->decToBinary(frame.data[4], b4);

    int highBin[4] = {b4[0], b4[1], b4[2], b4[3]};
    int lowBin[4] = {b4[4], b4[5], b4[6], b4[7]};
    int d4_high = this->binaryToDec(highBin);

    int d4_low = this->binaryToDec(lowBin);

    int v_data = frame.data[3] * 16 + d4_high;

    float v = (v_data * 90) / 4095. - 45;

    result[2] = v;

    int i_data = d4_low * 16 * 16 + frame.data[5];
    float i = (i_data * 36) / 4095. - 18;
    result[3] = i;

    return result;
}

void Actuator::decToBinary(int n, int *binaryNum) {
    for (int i = 7; i >= 0; i--)
    {
        binaryNum[i] = n % 2;
        n /= 2;
    }
}

int Actuator::binaryToDec(int *binaryNum) {
    int decimal = 0;

    for (int i = 0; i < 4; i++)
        decimal = decimal * 2 + binaryNum[i];

    return decimal;
}
