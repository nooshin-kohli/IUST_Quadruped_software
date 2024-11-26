#include "rt/rt_can.h"
#include <chrono>
#include <thread>


CANCommand* can_cmd = new CANCommand;
CANData* can_data = new CANData;


// only used for actual robot
const float abad_side_sign[4] = {-1.f, -1.f, 1.f, 1.f};
const float hip_side_sign[4] = {1.f, -1.f, 1.f, -1.f};
const float knee_side_sign[4] = {.6429f, -.6429f, .6429f, -.6429f};

// only used for actual robot
const float abad_offset[4] = {-0.0520f, -0.1361f, -0.6377f, -0.0291f};
const float hip_offset[4]  = {0.2349f, -1.8611f,  0.0812f,  -1.3335f};
const float knee_offset[4] = {-4.2211f, 3.6792f, -4.6734f, 3.8183f};




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

  

  FR.command(m1, (can_command->q_des_abad[0] / abad_side_sign[0]) + abad_offset[0],can_command->qd_des_abad[0]/abad_side_sign[0],can_command->kp_abad[0],can_command->kd_abad[0],can_command->tau_abad_ff[0]/abad_side_sign[0]);
  FL.command(m4, (can_command->q_des_abad[1] / abad_side_sign[1]) + abad_offset[1],can_command->qd_des_abad[1]/abad_side_sign[1],can_command->kp_abad[1],can_command->kd_abad[1],can_command->tau_abad_ff[1]/abad_side_sign[1]);
  RR.command(m7, (can_command->q_des_abad[2] / abad_side_sign[2]) + abad_offset[2],can_command->qd_des_abad[2]/abad_side_sign[2],can_command->kp_abad[2],can_command->kd_abad[2],can_command->tau_abad_ff[2]/abad_side_sign[2]);
  RL.command(m10, (can_command->q_des_abad[3]/ abad_side_sign[3]) + abad_offset[3],can_command->qd_des_abad[3]/abad_side_sign[3],can_command->kp_abad[3],can_command->kd_abad[3],can_command->tau_abad_ff[3]/abad_side_sign[3]);

  FR.command(m2, (can_command->q_des_hip[0] / hip_side_sign[0]) + hip_offset[0],can_command->qd_des_hip[0]/ hip_side_sign[0],can_command->kp_hip[0],can_command->kd_hip[0],can_command->tau_hip_ff[0]/ hip_side_sign[0]);
  FL.command(m5, (can_command->q_des_hip[1] / hip_side_sign[1]) + hip_offset[1],can_command->qd_des_hip[1]/ hip_side_sign[1],can_command->kp_hip[1],can_command->kd_hip[1],can_command->tau_hip_ff[1]/ hip_side_sign[1]);
  RR.command(m8, (can_command->q_des_hip[2] / hip_side_sign[2]) + hip_offset[2],can_command->qd_des_hip[2]/ hip_side_sign[2],can_command->kp_hip[2],can_command->kd_hip[2],can_command->tau_hip_ff[2]/ hip_side_sign[2]);
  RL.command(m11, (can_command->q_des_hip[3]/ hip_side_sign[3]) + hip_offset[3],can_command->qd_des_hip[3]/ hip_side_sign[3],can_command->kp_hip[3],can_command->kd_hip[3],can_command->tau_hip_ff[3]/ hip_side_sign[3]);

  FR.command(m3, (can_command->q_des_knee[0] / knee_side_sign[0]) + knee_offset[0],can_command->qd_des_knee[0]/ knee_side_sign[0],can_command->kp_knee[0],can_command->kd_knee[0],can_command->tau_knee_ff[0]/ knee_side_sign[0]);
  FL.command(m6, (can_command->q_des_knee[1] / knee_side_sign[1]) + knee_offset[1],can_command->qd_des_knee[1]/ knee_side_sign[1],can_command->kp_knee[1],can_command->kd_knee[1],can_command->tau_knee_ff[1]/ knee_side_sign[1]);
  RR.command(m9, (can_command->q_des_knee[2] / knee_side_sign[2]) + knee_offset[2],can_command->qd_des_knee[2]/ knee_side_sign[2],can_command->kp_knee[2],can_command->kd_knee[2],can_command->tau_knee_ff[2]/ knee_side_sign[2]);
  RL.command(m12, (can_command->q_des_knee[3]/ knee_side_sign[3]) + knee_offset[3],can_command->qd_des_knee[3]/ knee_side_sign[3],can_command->kp_knee[3],can_command->kd_knee[3],can_command->tau_knee_ff[3]/ knee_side_sign[3]);

  // printf("Motor response :%f\n", RL.cycle_responses[2][1]);
  

  can_response->q_abad[0] = (FR.cycle_responses[0][1] - abad_offset[0]) * abad_side_sign[0];
  can_response->q_hip[0] = (FR.cycle_responses[1][1] - hip_offset[0]) * hip_side_sign[0];
  can_response->q_knee[0] = (FR.cycle_responses[2][1] - knee_offset[0]) * knee_side_sign[0];

  can_response->q_abad[1]= (FL.cycle_responses[0][1] - abad_offset[1]) * abad_side_sign[1];
  can_response->q_hip[1]= (FL.cycle_responses[1][1] - hip_offset[1]) * hip_side_sign[1];
  can_response->q_knee[1]= (FL.cycle_responses[2][1] - knee_offset[1]) * knee_side_sign[1];

  can_response->q_abad[2]= (RR.cycle_responses[0][1] - abad_offset[2]) * abad_side_sign[2];
  can_response->q_hip[2]= (RR.cycle_responses[1][1] - hip_offset[2]) * hip_side_sign[2];
  can_response->q_knee[2]= (RR.cycle_responses[2][1] - knee_offset[2]) * knee_side_sign[2];


  can_response->q_abad[3] = (RL.cycle_responses[0][1] - abad_offset[3]) * abad_side_sign[3];
  can_response->q_hip[3] = (RL.cycle_responses[1][1] - hip_offset[3]) * hip_side_sign[3];
  can_response->q_knee[3] = (RL.cycle_responses[2][1] - knee_offset[3]) * knee_side_sign[3];

  ////// For offset calculation uncomment below:
  // printf("ABAD Q = [%f, %f, %f, %f]\n", FR.cycle_responses[0][1] , FL.cycle_responses[0][1], RR.cycle_responses[0][1],RL.cycle_responses[0][1]);
  // printf("HIP  Q = [%f, %f, %f, %f]\n", FR.cycle_responses[1][1] , FL.cycle_responses[1][1], RR.cycle_responses[1][1],RL.cycle_responses[1][1]);
  // printf("KNEE Q = [%f, %f, %f, %f]\n", FR.cycle_responses[2][1] , FL.cycle_responses[2][1], RR.cycle_responses[2][1],RL.cycle_responses[2][1]);
  
  
  // std::cout<<"new data:"<<std::endl;
  // // std::cout<<FR.cycle_responses[1][1]<<std::endl;
  // std::cout<<RL.cycle_responses[0][1]<<std::endl;
  // std::cout<<RL.cycle_responses[1][1]<<std::endl;
  // std::cout<<RR.cycle_responses[1][1]<<std::endl;
  // // std::cout<<RL.cycle_responses[1][1]<<std::endl;
  // std::cout<<"end of new data"<<std::endl;

  can_response->qd_abad[0] = FR.cycle_responses[0][2]* abad_side_sign[0];
  can_response->qd_hip[0] = FR.cycle_responses[1][2] * hip_side_sign[0];
  can_response->qd_knee[0] = FR.cycle_responses[2][2]* knee_side_sign[0];

  can_response->qd_abad[1] = FL.cycle_responses[0][2]* abad_side_sign[1];
  can_response->qd_hip[1] = FL.cycle_responses[1][2] * hip_side_sign[1];
  can_response->qd_knee[1] = FL.cycle_responses[2][2]* knee_side_sign[1]; 

  can_response->qd_abad[2] = RR.cycle_responses[0][2]* abad_side_sign[2];
  can_response->qd_hip[2] = RR.cycle_responses[1][2] * hip_side_sign[2];
  can_response->qd_knee[2] = RR.cycle_responses[2][2]* knee_side_sign[2];

  can_response->qd_abad[3] = RL.cycle_responses[0][2]* abad_side_sign[3];
  can_response->qd_hip[3] = RL.cycle_responses[1][2] * hip_side_sign[3];
  can_response->qd_knee[3] = RL.cycle_responses[2][2]* knee_side_sign[3];
  
  printf("----------------------------------Torques------------------------------------\n");
  printf("ABAD Q = [%f, %f, %f, %f]\n", FR.cycle_responses[0][3] , FL.cycle_responses[0][3], RR.cycle_responses[0][3],RL.cycle_responses[0][3]);
  printf("HIP  Q = [%f, %f, %f, %f]\n", FR.cycle_responses[1][3] , FL.cycle_responses[1][3], RR.cycle_responses[1][3],RL.cycle_responses[1][3]);
  printf("KNEE Q = [%f, %f, %f, %f]\n", FR.cycle_responses[2][3] , FL.cycle_responses[2][3], RR.cycle_responses[2][3],RL.cycle_responses[2][3]);
  printf("-----------------------------------------------------------------------------\n");



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