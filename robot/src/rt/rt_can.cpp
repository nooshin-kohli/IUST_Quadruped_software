#include "rt/rt_can.h"
#include <chrono>
#include <thread>


CANCommand* can_cmd = new CANCommand;
CANData* can_data = new CANData;

void CAN::init_can() {
  
  FR.enable(m1);
  FR.enable(m2);
  FR.enable(m3);
  printf("[IUSTHardware] Leg 1 is enabled!\n");

  FL.enable(m4);
  FL.enable(m5);
  FL.enable(m6);
  printf("[IUSTHardware] Leg 2 is enabled!\n");
  
  RR.enable(m7);
  RR.enable(m8);
  RR.enable(m9);
  printf("[IUSTHardware] Leg 3 is enabled!\n");
  
  RL.enable(m10);
  RL.enable(m11);
  RL.enable(m12);
  printf("[IUSTHardware] Leg 4 is enabled!\n");

  // Loop to initialize each motor command
  for (int i = 0; i < 4; i++) {
    // printf("just once???? \n");
    can_cmd->q_des_abad[i] = 0.0;
    can_cmd->q_des_hip[i] = 0.0;
    can_cmd->q_des_knee[i] = 0.0;

    can_cmd->qd_des_abad[i] = 0.0;
    can_cmd->qd_des_hip[i] = 0.0;
    can_cmd->qd_des_knee[i] = 0.0;

    can_cmd->kp_abad[i] = 0.0;
    can_cmd->kp_hip[i] = 0.0;
    can_cmd->kp_knee[i] = 0.0;

    can_cmd->kd_abad[i] = 0.0;
    can_cmd->kd_hip[i] = 0.0;
    can_cmd->kd_knee[i] = 0.0;

    can_cmd->tau_abad_ff[i] = 0.0;
    can_cmd->tau_hip_ff[i] = 0.0;
    can_cmd->tau_knee_ff[i] = 0.0;
  }

  // Loop to initialize each motor command
  for (int i = 0; i < 4; i++) {
    can_data->q_abad[i] = 0.0;
    can_data->q_hip[i] = 0.0;
    can_data->q_knee[i] = 0.0;
    can_data->qd_abad[i] = 0.0;
    can_data->qd_hip[i] = 0.0;
    can_data->qd_knee[i] = 0.0;
  }
}

void CAN::can_send_receive(CANCommand* can_command, CANData* can_response) {


  // for (int i(0); i<4; i++){
  //   printf("--------------KP:------------------\n");
  //   std::cout<< can_command->kp_abad[i] <<std::endl;
  //   std::cout<< can_command->kp_hip[i] <<std::endl;
  //   std::cout<< can_command->kp_knee[i] <<std::endl;
  //   printf("----------------QDES:---------------\n");
  //   std::cout<< can_command->q_des_abad[i] <<std::endl;
  //   std::cout<< can_command->q_des_hip[i] <<std::endl;
  //   std::cout<< can_command->q_des_knee[i] <<std::endl;
  //   printf("----------------------------------\n");

  // }

  

  FR.command(m1, can_command->q_des_abad[0],can_command->qd_des_abad[0],can_command->kp_abad[0],can_command->kd_abad[0],can_command->tau_abad_ff[0]);
  FL.command(m4, can_command->q_des_abad[1],can_command->qd_des_abad[1],can_command->kp_abad[1],can_command->kd_abad[1],can_command->tau_abad_ff[1]);
  RR.command(m7, can_command->q_des_abad[2],can_command->qd_des_abad[2],can_command->kp_abad[2],can_command->kd_abad[2],can_command->tau_abad_ff[2]);
  RL.command(m10, can_command->q_des_abad[3],can_command->qd_des_abad[3],can_command->kp_abad[3],can_command->kd_abad[3],can_command->tau_abad_ff[3]);

  FR.command(m2, can_command->q_des_hip[0],can_command->qd_des_hip[0],can_command->kp_hip[0],can_command->kd_hip[0],can_command->tau_hip_ff[0]);
  FL.command(m5, can_command->q_des_hip[1],can_command->qd_des_hip[1],can_command->kp_hip[1],can_command->kd_hip[1],can_command->tau_hip_ff[1]);
  RR.command(m8, can_command->q_des_hip[2],can_command->qd_des_hip[2],can_command->kp_hip[2],can_command->kd_hip[2],can_command->tau_hip_ff[2]);
  RL.command(m11, can_command->q_des_hip[3],can_command->qd_des_hip[3],can_command->kp_hip[3],can_command->kd_hip[3],can_command->tau_hip_ff[3]);

  FR.command(m3, can_command->q_des_knee[0],can_command->qd_des_knee[0],can_command->kp_knee[0],can_command->kd_knee[0],can_command->tau_knee_ff[0]);
  FL.command(m6, can_command->q_des_knee[1],can_command->qd_des_knee[1],can_command->kp_knee[1],can_command->kd_knee[1],can_command->tau_knee_ff[1]);
  RR.command(m9, can_command->q_des_knee[2],can_command->qd_des_knee[2],can_command->kp_knee[2],can_command->kd_knee[2],can_command->tau_knee_ff[2]);
  RL.command(m12, can_command->q_des_knee[3],can_command->qd_des_knee[3],can_command->kp_knee[3],can_command->kd_knee[3],can_command->tau_knee_ff[3]);


can_response->q_abad[0] = FR.cycle_responses[0][1];
can_response->q_hip[0] = FR.cycle_responses[1][1];
can_response->q_knee[0] = FR.cycle_responses[2][1];



  can_response->q_abad[1]= FL.cycle_responses[0][1];
  can_response->q_hip[1]= FL.cycle_responses[1][1];
  can_response->q_knee[1]= FL.cycle_responses[2][1];

  can_response->q_abad[2]= RR.cycle_responses[0][1];
  can_response->q_hip[2]= RR.cycle_responses[1][1];
  can_response->q_knee[2]= RR.cycle_responses[2][1];

  can_response->q_abad[3] = RL.cycle_responses[0][1];
  can_response->q_hip[3] = RL.cycle_responses[1][1];
  can_response->q_knee[3] = RL.cycle_responses[2][1];


  can_response->qd_abad[0] = FR.cycle_responses[0][2];
  can_response->qd_hip[0] = FR.cycle_responses[1][2];
  can_response->qd_knee[0] = FR.cycle_responses[2][2];

  can_response->qd_abad[1] = FL.cycle_responses[0][2];
  can_response->qd_hip[1] = FL.cycle_responses[1][2];
  can_response->qd_knee[1] = FL.cycle_responses[2][2];

  can_response->qd_abad[2] = RR.cycle_responses[0][2];
  can_response->qd_hip[2] = RR.cycle_responses[1][2];
  can_response->qd_knee[2] = RR.cycle_responses[2][2];

  can_response->qd_abad[3] = RL.cycle_responses[0][2];
  can_response->qd_hip[3] = RL.cycle_responses[1][2];
  can_response->qd_knee[3] = RL.cycle_responses[2][2];



  can_cmd = can_command;
  can_data = can_response;
  // std::cout<<can_data->q_abad[0]<<std::endl;
  // std::cout<<"---------"<<std::endl;
  // std::cout<<can_response->q_abad[0]<<std::endl;

}

void CAN::check_safety() {

    // if()
        // stop_can();
}

void CAN::stop_can() {

  FR.disable(m1);
  FR.disable(m2);
  FR.disable(m3);
  
  FL.disable(m4);
  FL.disable(m5);
  FL.disable(m6);
  
  RR.disable(m7);
  RR.disable(m8);
  RR.disable(m9);
  
  RL.disable(m10);
  RL.disable(m11);
  RL.disable(m12);

}


CANCommand* CAN::get_can_command() {
  // can_cmd = new actuator_command_t[12];
  


  // int m_ids[12] = {m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11, m12};


  return can_cmd;
}

CANData* CAN::get_can_data() {
  // can_data = new actuator_response_t[12];

  // int m_ids[12] = {m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11, m12};



  // can_data->id = m1;
  return can_data;
}