
#include "IUST_Controller.hpp"

void IUST_Controller::initializeController() {
  debug_iter = 0;
  first_time = true;
}
void IUST_Controller::checkMaxPostureExceeded(float position[12], float Q_home[12]) {
  bool exceeded = false;
  const float tolerance = 0.1; // Define tolerance

  for (int i = 0; i < 4; i++) { // For each leg
    for (int j = 0; j < 3; j++) { // For each joint in a leg
      int index = 3 * i + j;
      float position_motor = position[index];
      float maxLimit = Q_home[index];

      // Check if exceeding q_home limit with tolerance
      if ((maxLimit >= 0 && (position_motor > maxLimit + tolerance)) ||
          (maxLimit < 0 && (position_motor < maxLimit - tolerance))) {
        printf("[WARNING] Joint %d of leg %d exceeded its limit. Position: %f, Limit: %f\n", j, i, position_motor, maxLimit);
        exceeded = true;
      }
    }
  }

  if (exceeded) {
    printf("[ERROR] Exceeded maximum posture defined in q_home. Stopping the robot.\n");
    safety = false;
    exit(0); // Stop the program or take another appropriate action
  }
  
}
void IUST_Controller::calculateQHome(const float q_initial[12]) {
  // Initialize q_home with values from q_ini
  for (int i = 0; i < 12; i++) {
    q_home[i] = q_initial[i];
  }

  // Modify specific indices as described
  q_home[1] = q_initial[1] + 1.7;
  q_home[4] = q_initial[4] - 1.7;
  q_home[7] = q_initial[7] + 1.7;
  q_home[10] = q_initial[10] - 1.7;

  q_home[2] = q_initial[2] - 1.3;
  q_home[5] = q_initial[5] + 1.3;
  q_home[8] = q_initial[8] - 1.3;
  q_home[11] = q_initial[11] + 1.3;


  printf("[IUST Controller] I have the Q_home now.\n");
}



void IUST_Controller::runController() {

  Mat3<float> kpMat;
  Mat3<float> kdMat;
  Mat3<float> kdC;
  Mat3<float> kpC;
  
  kpMat << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  kdMat << 0, 0, 0, 0, 0, 0, 0, 0, 0;

  kpC << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  kdC << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  // kpMat << userParameters.kp, 0, 0, 0,  userParameters.kp, 0, 0, 0,  userParameters.kp;
  // kdMat <<  userParameters.kd, 0, 0, 0, userParameters.kd, 0, 0, 0, userParameters.kd;
  if (first_time==true){
    for (int i(0);i<4;i++){
    _legController->commands[i].qDes[0] = 0.0;
    _legController->commands[i].qDes[1] = 0.0;
    _legController->commands[i].qDes[2] = 0.0;

    _legController->commands[i].qdDes[0] = 0.0;
    _legController->commands[i].qdDes[1] = 0.0;
    _legController->commands[i].qdDes[2] = 0.0;

    _legController->commands[i].kpJoint = kpMat;
    _legController->commands[i].kdJoint = kdMat;

    _legController->commands[i].kpCartesian = kpC;
    _legController->commands[i].kdCartesian = kdC;

  }

  for (int i(0); i<4; i++){
    q_now[3*i] = _legController->datas[i].q[0]; 
    q_now[3*i+1] = _legController->datas[i].q[1];
    q_now[3*i+2] = _legController->datas[i].q[2];

    /////////////////////////////////////////////////
    q_ini_resp[3*i] = _legController->datas[i].q[0]; 
    q_ini_resp[3*i+1] = _legController->datas[i].q[1];
    q_ini_resp[3*i+2] = _legController->datas[i].q[2];
  }
  
  printf("ABAD Q = [%f, %f, %f, %f]\n", _legController->datas[0].q[0], _legController->datas[1].q[0], _legController->datas[2].q[0], _legController->datas[3].q[0]);
  printf("HIP  Q = [%f, %f, %f, %f]\n", _legController->datas[0].q[1], _legController->datas[1].q[1], _legController->datas[2].q[1], _legController->datas[3].q[1]);
  printf("KNEE Q = [%f, %f, %f, %f]\n", _legController->datas[0].q[2], _legController->datas[1].q[2], _legController->datas[2].q[2], _legController->datas[3].q[2]);
  
  
  // for (int i(0); i<12; i++){
  //   if (q_now[2]>-1.2 && q_now[2]<-0.015){
  //     q_ini[2] = -1.0146f;
  //     // printf("I have changed the start of the motor 3!!!\n");
  //     // printf("the response was: %f and the new homing is %f.\n", q_now[2], q_ini[2]);
  //   }
  //   if (q_now[9]>-1.2 && q_now[9]<-0.015){
  //     q_ini[9] = -1.0175f;
  //     // printf("[IUST Controller] I have changed the homing of the motor 9.\n");
  //   }
  //   if (abs(q_now[i]-q_ini[i])>0.1){
  //     printf("Motor %d is not safe to start\n", i);
  //     safety = false;
  //   }
  // }
  // calculateQHome(q_ini);
  // for (int i = 0; i < 12; i++) {
  //   printf("q_home[%d] = %f\n", i, q_home[i]);
  // }
  first_time = true;



  }
  else{
    if (safety==true){

    for (int i(0);i<4;i++){
    _legController->commands[i].qDes[0] = 0.0;
    _legController->commands[i].qDes[1] = 0.0;
    _legController->commands[i].qDes[2] = 0.0;

    _legController->commands[i].qdDes[0] = 0.0;
    _legController->commands[i].qdDes[1] = 0.0;
    _legController->commands[i].qdDes[2] = 0.0;

    _legController->commands[i].kpJoint = kpMat;
    _legController->commands[i].kdJoint = kpMat;

    _legController->commands[i].kpCartesian = kpC;
    _legController->commands[i].kdCartesian = kdC;

  }
  // printf("I'm in the loop!!!!\n");
  for (int i(0); i<4; i++){
    q_now[3*i] = _legController->datas[i].q[0]; 
    q_now[3*i+1] = _legController->datas[i].q[1];
    q_now[3*i+2] = _legController->datas[i].q[2];
  }

  if (debug_iter % 1000 == 0) {
    printf("[INFO]  IUST controller is running!\n");
    printf("Iteration stamp:\t%d\n",(int)debug_iter);
    printf("--------------------_legController--------------------------\n");
    printf("ABAD Q = [%f, %f, %f, %f]\n", _legController->datas[0].q[0], _legController->datas[1].q[0], _legController->datas[2].q[0], _legController->datas[3].q[0]);
    printf("HIP  Q = [%f, %f, %f, %f]\n", _legController->datas[0].q[1], _legController->datas[1].q[1], _legController->datas[2].q[1], _legController->datas[3].q[1]);
    printf("KNEE Q = [%f, %f, %f, %f]\n", _legController->datas[0].q[2], _legController->datas[1].q[2], _legController->datas[2].q[2], _legController->datas[3].q[2]);
    printf("ABAD QD = [%f, %f, %f, %f]\n", _legController->datas[0].qd[0], _legController->datas[1].qd[0], _legController->datas[2].qd[0], _legController->datas[3].qd[0]);
    printf("HIP  QD = [%f, %f, %f, %f]\n", _legController->datas[0].qd[1], _legController->datas[1].qd[1], _legController->datas[2].qd[1], _legController->datas[3].qd[1]);
    printf("KNEE QD = [%f, %f, %f, %f]\n", _legController->datas[0].qd[2], _legController->datas[1].qd[2], _legController->datas[2].qd[2], _legController->datas[3].qd[2]);
    printf("-------------------COMMAND------------------------\n");
    // printf("ABAD DES Q = [%f, %f, %f, %f]\n", _spiCommand.q_des_abad[0],_spiCommand.q_des_abad[1],_spiCommand.q_des_abad[2],_spiCommand.q_des_abad[3]);
    // printf("HIP  DES Q = [%f, %f, %f, %f]\n", _spiCommand.q_des_hip[0],_spiCommand.q_des_hip[1],_spiCommand.q_des_hip[2],_spiCommand.q_des_hip[3]);
    // printf("KNEE DES Q = [%f, %f, %f, %f]\n", _spiCommand.q_des_knee[0],_spiCommand.q_des_knee[1],_spiCommand.q_des_knee[2],_spiCommand.q_des_knee[3]);
    // printf("KP A= [%f, %f, %f, %f]\n", _spiCommand.kp_abad[0],_spiCommand.kp_abad[1],_spiCommand.kp_abad[2],_spiCommand.kp_abad[3]);
    // printf("KP H= [%f, %f, %f, %f]\n", _spiCommand.kp_hip[0],_spiCommand.kp_hip[1],_spiCommand.kp_hip[2],_spiCommand.kp_hip[3]);
    // printf("KP K= [%f, %f, %f, %f]\n", _spiCommand.kp_knee[0],_spiCommand.kp_knee[1],_spiCommand.kp_knee[2],_spiCommand.kp_knee[3]);
    // printf("KD A= [%f, %f, %f, %f]\n", _spiCommand.kd_abad[0],_spiCommand.kd_abad[1],_spiCommand.kd_abad[2],_spiCommand.kd_abad[3]);
    // printf("KD H= [%f, %f, %f, %f]\n", _spiCommand.kd_hip[0],_spiCommand.kd_hip[1],_spiCommand.kd_hip[2],_spiCommand.kd_hip[3]);
    // printf("KD K= [%f, %f, %f, %f]\n", _spiCommand.kd_knee[0],_spiCommand.kd_knee[1],_spiCommand.kd_knee[2],_spiCommand.kd_knee[3]);
    // printf("ABAD DES T = [%f, %f, %f, %f]\n", _spiCommand.tau_abad_ff[0],_spiCommand.tau_abad_ff[1],_spiCommand.tau_abad_ff[2],_spiCommand.tau_abad_ff[3]);
    // printf("HIP  DES T = [%f, %f, %f, %f]\n", _spiCommand.tau_hip_ff[0],_spiCommand.tau_hip_ff[1],_spiCommand.tau_hip_ff[2],_spiCommand.tau_hip_ff[3]);
    // printf("KNEE DES T = [%f, %f, %f, %f]\n", _spiCommand.tau_knee_ff[0],_spiCommand.tau_knee_ff[1],_spiCommand.tau_knee_ff[2],_spiCommand.tau_knee_ff[3]);
    printf("--------------------------------------------------\n\n\n");
  }

  debug_iter++;

  
    
  }
  
  else{
    printf("Not in the loop !!!!!!\n");
  }

  }
  


}

void IUST_Controller::Standup(const size_t & curr_iter){
  Mat3<float> kpMat;
  Mat3<float> kdMat;
  Mat3<float> kdC;
  Mat3<float> kpC;

  
  kpC << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  kdC << 0, 0, 0, 0, 0, 0, 0, 0, 0;

  kpMat << 60, 0, 0, 0, 60, 0, 0, 0, 20;
  kdMat << 1, 0, 0, 0, 1 , 0, 0, 0, 0.5;
  
  for (int i(0); i<4;i++){
    _legController->commands[i].kpJoint = kpMat;
    _legController->commands[i].kdJoint = kdMat;
  }
  if (curr_iter<4000){
    printf("[IUST Controller]: fixing robot in It's place.\n");
    // printf("[IUST Controller] Homing Hip Joint! \n");
    for (int i(0); i<4; i++){
      _legController->commands[i].kpJoint = kpMat;
      _legController->commands[i].kdJoint = kdMat;
      _legController->commands[i].kpCartesian = kpC;
      _legController->commands[i].kdCartesian = kdC;

      _legController->commands[i].qDes[0] = q_now[3*i];
      _legController->commands[i].qDes[1] = q_now[3*i+1];
      _legController->commands[i].qDes[2] = q_now[3*i+2];
      _legController->commands[i].qdDes[0] = 0.0;
      _legController->commands[i].qdDes[1] = 0.0;
      _legController->commands[i].qdDes[2] = 0.0;
   
    }
  }else if (curr_iter==4000){

    
    kpMat << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    kdMat << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  for (int i(0); i<4;i++){
    _legController->commands[i].kpJoint = kpMat;
    _legController->commands[i].kdJoint = kdMat;
  }
  for (int i(0); i<4; i++){
    /////////////////////////////////////////////////
    q_ini_hip[3*i] = _legController->datas[i].q[0]; 
    q_ini_hip[3*i+1] = _legController->datas[i].q[1];
    q_ini_hip[3*i+2] = _legController->datas[i].q[2];
  }
    printf("[IUST Controller] Done homing the ABAD joints!!!\n");
  }else if(curr_iter>4000 and curr_iter<6000){
    kpMat << 60, 0, 0, 0, 60, 0, 0, 0, 20;
    kdMat << 1, 0, 0, 0, 1, 0, 0, 0, 0.5;
  for (int i(0); i<4;i++){
    _legController->commands[i].kpJoint = kpMat;
    _legController->commands[i].kdJoint = kdMat;
  }
    q_des[1] = leg_Interpolation(curr_iter-4000, 2000, q_ini_hip[1], q_ini_hip[1] + 1.6);
    q_des[4] = leg_Interpolation(curr_iter-4000, 2000, q_ini_hip[4], q_ini_hip[4] - 1.6);
    q_des[7] = leg_Interpolation(curr_iter-4000, 2000, q_ini_hip[7], q_ini_hip[7] + 1.6);
    q_des[10] = leg_Interpolation(curr_iter-4000, 2000, q_ini_hip[10], q_ini_hip[10] - 1.6);
    printf("DES of HIP: %f\n", q_des[1]);

    for (int i(0);i<4;i++){
    _legController->commands[i].qDes[0] = q_ini_hip[3*i];
    _legController->commands[i].qDes[1] = q_des[3*i+1];
    _legController->commands[i].qDes[2] = q_ini_hip[3*i+2];
    _legController->commands[i].qdDes[0] = 0.0;
    _legController->commands[i].qdDes[1] = 0.0;
    _legController->commands[i].qdDes[2] = 0.0;

    _legController->commands[i].kpJoint = kpMat;
    _legController->commands[i].kdJoint = kdMat;

    _legController->commands[i].kpCartesian = kpC;
    _legController->commands[i].kdCartesian = kdC;
}

  }else if(curr_iter==6000){
    kpMat << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    kdMat << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  
  for (int i(0); i<4;i++){
    _legController->commands[i].kpJoint = kpMat;
    _legController->commands[i].kdJoint = kdMat;
  }
   for (int i(0); i<4; i++){
    /////////////////////////////////////////////////
    q_ini_knee[3*i] = _legController->datas[i].q[0]; 
    q_ini_knee[3*i+1] = _legController->datas[i].q[1];
    q_ini_knee[3*i+2] = _legController->datas[i].q[2];
  }
    printf("[IUST Controller] Done homing the HIP joints!!!\n");
  }else if (curr_iter>6000 && curr_iter<8000){
    kpMat << 60, 0, 0, 0, 60, 0, 0, 0, 40;
    kdMat << 1, 0, 0, 0, 1, 0, 0, 0, 0.5;
  for (int i(0); i<4;i++){
    _legController->commands[i].kpJoint = kpMat;
    _legController->commands[i].kdJoint = kdMat;
  }
    q_des[2] = leg_Interpolation(curr_iter-6000, 2000, q_ini_knee[2], q_ini_knee[2] - 1.2);
    q_des[5] = leg_Interpolation(curr_iter-6000, 2000, q_ini_knee[5], q_ini_knee[5] + 1.2);
    q_des[8] = leg_Interpolation(curr_iter-6000, 2000, q_ini_knee[8], q_ini_knee[8] - 1.2);
    q_des[11] = leg_Interpolation(curr_iter-6000, 2000, q_ini_knee[11], q_ini_knee[11] + 1.2);
    printf("DES of Knee: %f\n", q_des[2]);

    for (int i(0);i<4;i++){
    _legController->commands[i].qDes[0] = q_ini_knee[3*i];
    _legController->commands[i].qDes[1] = q_ini_knee[3*i+1];
    _legController->commands[i].qDes[2] = q_des[3*i+2];
    _legController->commands[i].qdDes[0] = 0.0;
    _legController->commands[i].qdDes[1] = 0.0;
    _legController->commands[i].qdDes[2] = 0.0;

    _legController->commands[i].kpJoint = kpMat;
    _legController->commands[i].kdJoint = kdMat;

    _legController->commands[i].kpCartesian = kpC;
    _legController->commands[i].kdCartesian = kdC;
}
  }else if(curr_iter == 8000){
    printf("[IUST Controller] Done homing the Knee joints!!!\n");
    exit(0);
    kpMat << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    kdMat << 0, 0, 0, 0, 0, 0, 0, 0, 0;

    for (int i(0); i<4;i++){
      _legController->commands[i].kpJoint = kpMat;
      _legController->commands[i].kdJoint = kdMat;
    }
  }else{
    exit(0);
  }

}



void IUST_Controller::Open_Calf_Joint(const size_t & curr_iter){

  Mat3<float> kpMat;
  Mat3<float> kdMat;
  Mat3<float> kdC;
  Mat3<float> kpC;

  
  kpC << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  kdC << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  for (int i(0); i<4; i++){

    _legController->commands[i].kpJoint = kpMat;
    _legController->commands[i].kdJoint = kpMat;
  }

   q_des[2] = leg_Interpolation(curr_iter, 1000, q_ini_resp[2], q_ini_resp[2] - 0.3);
   q_des[5] = leg_Interpolation(curr_iter, 1000, q_ini_resp[5], q_ini_resp[5] + 0.3);
   q_des[8] = leg_Interpolation(curr_iter, 1000, q_ini_resp[8], q_ini_resp[8] - 0.3);
   q_des[11] = leg_Interpolation(curr_iter, 1000, q_ini_resp[11], q_ini_resp[11] + 0.3);

  //  float q_tempp = leg_Interpolation(curr_iter, 1000, q_now[2], q_ini_resp[2] - 0.3);

  // printf("res: %f\n",q_tempp);
  // printf("now: %f\n",q_des[2]);

  if (curr_iter>1000){
    kpMat << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    kdMat << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    printf("DONE INTERPOLATE!!!\n");
    for (int i(0);i<4;i++){
    _legController->commands[i].qDes[0] = 0.0;
    _legController->commands[i].qDes[1] = 0.0;
    _legController->commands[i].qDes[2] = 0.0;

    _legController->commands[i].qdDes[0] = 0.0;
    _legController->commands[i].qdDes[1] = 0.0;
    _legController->commands[i].qdDes[2] = 0.0;

    _legController->commands[i].kpJoint = kpMat;
    _legController->commands[i].kdJoint = kdMat;

    _legController->commands[i].kpCartesian = kpC;
    _legController->commands[i].kdCartesian = kdC;
    }
    exit(0);
   }
   else{
    kpMat << 20, 0, 0, 0, 20, 0, 0, 0, 10;
    kdMat << 0.5, 0, 0, 0, 0.5, 0, 0, 0, 0.5;

    for (int i(0);i<4;i++){
    _legController->commands[i].qDes[0] = q_now[3*i];
    _legController->commands[i].qDes[1] = q_now[3*i+1];
    _legController->commands[i].qDes[2] = q_des[3*i+2];

    // if (i==1){
    //   printf("I'm commanding %f to motor 2 of leg 1\n",q_des[5]);
    //   _legController->commands[1].qDes[2] = q_des[5];
    // }else{
    //   _legController->commands[i].qDes[2] = q_now[3*i+2];
    // }
    // _legController->commands[i].qDes[2] = 0.0;

    _legController->commands[i].qdDes[0] = 0.0;
    _legController->commands[i].qdDes[1] = 0.0;
    _legController->commands[i].qdDes[2] = 0.0;

    _legController->commands[i].kpJoint = kpMat;
    _legController->commands[i].kdJoint = kdMat;

    _legController->commands[i].kpCartesian = kpC;
    _legController->commands[i].kdCartesian = kdC;
}
     
   }

  //  printf("leg 1: Starting from %f to %f, currently at %f. \n  ", q_now[2], q_now[2]-0.2, q_des[2]);
  //  printf("leg 2: Starting from %f to %f, currently at %f. \n  ", q_now[5], q_now[5]+0.2, q_des[5]);
  //  printf("leg 3: Starting from %f to %f, currently at %f. \n  ", q_now[8], q_now[8]-0.2, q_des[8]);
  //  printf("leg 4: Starting from %f to %f, currently at %f. \n  ", q_now[11], q_now[11]+0.2, q_des[11]);
   




}

float IUST_Controller::leg_Interpolation(const size_t & curr_iter, size_t max_iter,
                                        const float & ini, const float & fin) {

    float a(0.f);
    float b(1.f);

    // interpolating
    if(curr_iter <= max_iter) {
      b = (float)curr_iter / (float)max_iter;
      a = 1.f - b;
    }

    // compute setpoints
    float inter_pos = a * ini + b * fin;

    // send control commands
    return inter_pos;
}