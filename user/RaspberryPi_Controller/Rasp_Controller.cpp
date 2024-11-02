// #include "Rasp_Controller.hpp"
// #include "cppTypes.h"

// float tau_ff[12];
// float f_ff[12];
// float q_des[12];
// float qd_des[12];
// float p_des[12];
// float v_des[12];
// float kp_cartesian[12];
// float kd_cartesian[12];
// float kp_joint[12];
// float kd_joint[12];
// bool safetyCheck;

// float q[12] = {-0.6435f, -0.3187f, -0.6927f,
//                -0.7594f, -0.2840f, -0.1709f,
//                -0.8087f, -0.4086f, -0.9566f,
//                -0.5681f,  0.0146f -1, -0.7188f};
// float qd[12];
// float p[12];
// float v[12];
// float tau_est[12];

// // Handler handlerObject;
// // lcm::LCM rasp_lcm("udpm://224.0.55.55:5001?ttl=225");

// void Rasp_Controller::initializeController() {
//   // Real Robot
//   for(int i(0); i < 12; i++){
//     tau_ff[i] = 0;
//     f_ff[i] = 0;
//     q_des[i] = 0;
//     qd_des[i] = 0;
//     p_des[i] = 0;
//     v_des[i] = 0;
//     kp_cartesian[i] = 0;
//     kd_cartesian[i] = 0;
//     kp_joint[i] = 0;
//     kd_joint[i] = 0;
//     safetyCheck = true;
//   }

//   // Simulation
//   _legController->_maxTorque = userParameters.max_tau;
//   _legController->_legsEnabled = true;

//   // Both
//   leg_enable << 0, 0, 0, 0;
//   desGamepadCommand << 0.0, 0.0, 0.0;

//     if(userParameters.calibrate > 0.4) {  // This param is only used in TI board in Cheetah3
//         _legController->_calibrateEncoders = userParameters.calibrate;
//     } else {
//         if (userParameters.zero > 0.5) {  //　This param is only used in TI board in Cheetah3
//             _legController->_zeroEncoders = true;
//         } else {
//             _legController->_zeroEncoders = false;
//         }
//     }

//   //q_ini[] = {-0.6435f, -0.052f, -0.4289f, -0.7594f, -0.5449f, -0.9682f, -0.8087f, -0.6638f, -0.2434f, -0.5681f, -0.7652f, -0.0636f};
  
//   printf("I'm doing OK!\n");
// }

// void Rasp_Controller::updateRaspCommand(){
//   motion_iter++;
//   // Get robot initial joint position
//     if (j_iter < 100 && currentState == FOLDLEG) {
//       // Real Rboot
//       for(int i(0); i < 12; i++) {
//         tau_ff[i] = 0;
//         f_ff[i] = 0;
//         q_des[i] = 0;
//         qd_des[i] = 0;
//         p_des[i] = 0;
//         v_des[i] = 0;
//         kp_cartesian[i] = 0;
//         kd_cartesian[i] = 0;
//         kp_joint[i] = 0;
//         kd_joint[i] = 0;
//       }
//     } else if (j_iter >= 100 && currentState == FOLDLEG) {
//       if (motion_iter > wait_iter) {
//         motion_iter = 0;
//         currentState = HIP_HOMING;
//         printf("[BREAKING NEWS]: Waiting finished...\n");
//       } else {
//         for(int i(0); i < 12; i++) {
//           q_des[i] = rx_ini[i];
//           kp_joint[i] = 80;
//           kd_joint[i] = 0.5;
//         }
//       }
//     }
//     else if (currentState == HIP_HOMING) {
//       if (motion_iter > hip_iter) {
//         motion_iter = 0;
//         currentState = KNEE_HOMING;
//         printf("[BREAKING NEWS]: Hip homing finished...\n");
//       } else {
//         for(int i(0); i < 4; i++) {
//           q_des[3 * i + 1] = leg_Interpolation(motion_iter, hip_iter, rx_ini[3 * i + 1], q_home[3 * i + 1]);
//         }
//       }
//     }
//     else if (currentState == KNEE_HOMING) {
//       if (motion_iter > knee_iter) {
//         motion_iter = 0;
//         currentState = JOINT;
//         printf("[BREAKING NEWS]: Knee homing finished...\n");
//       } else {
//         for(int i(0); i < 4; i++) {
//           q_des[3 * i + 2] = leg_Interpolation(motion_iter, knee_iter, rx_ini[3 * i + 2], q_home[3 * i + 2]);
//         }
//       }
//     }
//     else if (currentState == JOINT) {
//       for(int i(0); i < 4; i++) {
//         if(leg_enable[i]) {
//           if (motion_iter < 100) {
//             if (i == 2 || i == 3) {
//               if (i==2) {
//                 q_des[5] = leg_Interpolation(motion_iter, 100, q_home[5], q_home[5] - 0.4);
//               } else {
//                 q_des[2] = leg_Interpolation(motion_iter, 100, q_home[2], q_home[2] + 0.4);
//               }
              
//             } else {
//               if (i==0){
//                 q_des[11] = leg_Interpolation(motion_iter, 100, q_home[11], q_home[11] - 0.4);
//               }else{
//                 q_des[8] = leg_Interpolation(motion_iter, 100, q_home[8], q_home[8] + 0.4);
//               }
//             }
//           } else {
//             updateGamepadCommand();
//             q_des[3*i] = q_home[3*i] + desGamepadCommand[0];
//             q_des[3*i + 1] = q_home[3*i + 1] + desGamepadCommand[1];
//             q_des[3*i + 2] = q_home[3*i + 2] + desGamepadCommand[2];
//           }
//         }
//       }
//     }
//     else if (currentState == BODY_POS){

//       if (motion_iter<100){
//         q_des[2] = leg_Interpolation(motion_iter, 100, q_home[2],q_home[2]+0.2);
//         q_des[5] = leg_Interpolation(motion_iter, 100, q_home[5], q_home[5]-0.2);
//         q_des[8] = leg_Interpolation(motion_iter, 100, q_home[8], q_home[8] - 0.2);
//         q_des[11] = leg_Interpolation(motion_iter, 100, q_home[11], q_home[11] + 0.2);
//       }
//       else if (motion_iter > 100 && motion_iter< 200)
//       {
//         q_des[2] = leg_Interpolation(motion_iter-100, 100, q_home[2]+0.2,q_home[2]);
//         q_des[5] = leg_Interpolation(motion_iter-100, 100, q_home[5]-0.2, q_home[5]);
//         q_des[8] = leg_Interpolation(motion_iter-100, 100, q_home[8]-0.2, q_home[8]);
//         q_des[11] = leg_Interpolation(motion_iter-100, 100, q_home[11]+0.2, q_home[11]);
//       }else if (motion_iter>200 && motion_iter<300)
//       {
//         q_des[2] = leg_Interpolation(motion_iter-200, 100, q_home[2],q_home[2]-0.2);
//         q_des[5] = leg_Interpolation(motion_iter-200, 100, q_home[5], q_home[5]+0.2);
//         q_des[8] = leg_Interpolation(motion_iter-200, 100, q_home[8], q_home[8] + 0.2);
//         q_des[11] = leg_Interpolation(motion_iter-200,100, q_home[11], q_home[11] - 0.2);
//       }else if (motion_iter>300 && motion_iter<400)
//       {
//         q_des[2] = leg_Interpolation(motion_iter-300, 100, q_home[2]-0.2,q_home[2]);
//         q_des[5] = leg_Interpolation(motion_iter-300, 100, q_home[5]+0.2, q_home[5]);
//         q_des[8] = leg_Interpolation(motion_iter-300, 100, q_home[8]+0.2, q_home[8]);
//         q_des[11] = leg_Interpolation(motion_iter-300,100, q_home[11]-0.2, q_home[11]);
//       }else if(motion_iter==400){
//         if (m<3){
//           motion_iter = 0;
//           m++;
//         }else{
//           currentState = JOINT;
//           m=0;
//         }
//       }
//     }
//     else if (currentState == HI) {
//       if (motion_iter < 100) {
//         q_des[11] = leg_Interpolation(motion_iter, 100, q_home[11], q_home[11] - 0.3);
//         q_des[0] = leg_Interpolation(motion_iter, 100, q_home[0], q_home[0] + 0.1);
//       } else if (motion_iter > 100 && motion_iter < 500) {
//         q_des[2] = leg_Interpolation(motion_iter - 100, 400, q_home[2], q_home[2] - 0.5);
//         q_des[5] = leg_Interpolation(motion_iter - 100, 400, q_home[5], q_home[5] + 0.5);
//         q_des[8] = leg_Interpolation(motion_iter - 100, 400, q_home[8], q_home[8] + 0.5);
//         q_des[11] = leg_Interpolation(motion_iter, 500, q_home[11] - 0.3, q_home[11] - 0.5);
//       } else if (motion_iter > 500 && motion_iter < 1000) {
//         q_des[5] = leg_Interpolation(motion_iter - 500, 500, q_home[5] + 0.5, q_home[5] + 0.3);
//         q_des[4] = leg_Interpolation(motion_iter - 500, 500, q_home[4], q_home[4] - 1.5);
//       } else if (motion_iter > 1000 && motion_iter < 1050) {
//         q_des[5] = leg_Interpolation(motion_iter - 1000, 50, q_home[5] + 0.3, q_home[5] + 0.1);
//       } else if (motion_iter > 1050 && motion_iter < 1100) {
//         q_des[5] = leg_Interpolation(motion_iter - 1050, 50, q_home[5] + 0.1, q_home[5] + 0.3);
//       } else if (motion_iter > 1100 && motion_iter < 1150) {
//         q_des[5] = leg_Interpolation(motion_iter - 1100, 50, q_home[5] + 0.3, q_home[5] + 0.1);
//       } else if (motion_iter > 1150 && motion_iter < 1200) {
//         q_des[5] = leg_Interpolation(motion_iter - 1150, 50, q_home[5] + 0.1, q_home[5] + 0.3);
//       }
//     }
//     else if (currentState == GO_HOME) {
//       if (motion_iter > shutdown_iter) {
//         motion_iter = 0;
//         currentState = SHUTDOWN;
//         printf("[BREAKING NEWS]: I'm going to sleep\n");
//       } else {
//         for (int i(0); i < 4; i++) {
//           q_des[3 * i + 2] = leg_Interpolation(motion_iter, shutdown_iter, q_home[3 * i + 2], rx_ini[3 * i + 2]);
//         }
//       }
//     }
//     else if (currentState == RECOVERY_STAND) {
//       if (yekBar) {
//         for(int i(0); i < 12; i++) {
//           q_now[i] = q[i];
//         }
//         yekBar = false;
//       }
//       if (motion_iter > recovery_iter) {
//         motion_iter = 0;
//         currentState = JOINT;
//         yekBar = true;
//         leg_enable << 0, 0, 0, 0;
//         printf("[BREAKING NEWS]: Back to home!\n");
//       } else {
//         for(int i(0); i < 12; i++) {
//           q_des[i] = leg_Interpolation(motion_iter, recovery_iter, q_now[i], q_home[i]);
//         }
//       }

//     }
//     else if(currentState == SHUTDOWN) {
//       //printf("I'm Done!!!\n");
//     }
//     //printf("Total tau: %f\n", abs(tau_est[0]) + abs(tau_est[1]) + abs(tau_est[2]) + abs(tau_est[3]) + abs(tau_est[4]) + abs(tau_est[5]) + abs(tau_est[6]) + abs(tau_est[7]) + abs(tau_est[8]) + abs(tau_est[9]) + abs(tau_est[10]) + abs(tau_est[11]));

// }


// void Rasp_Controller::runController() {
//   ++j_iter;
//   legDetection();

//   // Real Robot
//   updateRaspCommand();
//   for(int i(0); i < 12; i++) {
//     COMMAND.tau_ff[i] = tau_ff[i];
//     COMMAND.f_ff[i] = f_ff[i];
//     COMMAND.q_des[i] = q_des[i];
//     COMMAND.qd_des[i] = qd_des[i];
//     COMMAND.p_des[i] = p_des[i];
//     COMMAND.v_des[i] = v_des[i];
//     COMMAND.kp_cartesian[i] = kp_cartesian[i];
//     COMMAND.kd_cartesian[i] = kd_cartesian[i];
//     COMMAND.kp_joint[i] = kp_joint[i];
//     COMMAND.kd_joint[i] = kd_joint[i];
//     COMMAND.safety = safetyCheck;
//   }
  
//   for (int i(0); i<4, i++){
//     _legController->commands[i].qDes[0] = q_des[3*i];
//     _legController->commands[i].qDes[1] = q_des[3*i+1];
//     _legController->commands[i].qDes[2] = q_des[3*i+2];
//   }

//   // rasp_lcm.publish("command", &COMMAND);

//   // rasp_lcm.subscribe("data", &Handler::handleMessage, &handlerObject);
//   // rasp_lcm.handle();

//   if(!FirstTimeSafetyCheck) {

//     printf("In the first time, Checking if robot is safe for standing up!!!!\n");

//     for (int i(0); i<4; i++){
//       q_now[i] = _legController->datas[i].q[0];
//       q_now[3*i+1] = _legController->datas[i].q[1];
//       q_now[3*i+2] = _legController->datas[i].q[2];
//     }


//     for (int i(0); i<12; i++){
//       if (abs(q_now[i]-q_ini[i])>0.1){
//         safetyCheck = false;
//         if (safetyCheck==false){
//           _legController->commands[i].qDes[0] = 0.0;
//           _legController->commands[i].qDes[1] = 0.0;
//           _legController->commands[i].qDes[2] = 0.0;
//           _legController->commands[i].kpJoint = Mat3<T>::Zero();
//           _legController->commands[i].kpJoint = Mat3<T>::Zero();
//           _legController->commands[i].kpJoint = Mat3<T>::Zero();
//           _legController->commands[i].kdJoint = Mat3<T>::Zero();
//           _legController->commands[i].kdJoint = Mat3<T>::Zero();
//           _legController->commands[i].kdJoint = Mat3<T>::Zero();
//           printf("[DANGER!!!], robot WAS NOT in the initial desired position to start!!!!!!");

//         }
//       }
//     }
    
//     // for(int i(0); i < 12; i++) {
//     //   if (i == 10) {
//     //     if (abs(q[i]) < 0.5) {
//     //       q_ini[i] = 0.0146f;
//     //     } else {
//     //       q_ini[i] = -0.9854f;
//     //     }
//     //   }
//     //   if(abs(q[i] - q_ini[i]) > 0.1) {
//     //     safetyCheck = false;
//     //     printf("[WARNING]: Motor %d with %f is NOT home but q_home is %f!!!\n", i + 1, q[i], q_ini[i]);
//     //     break;
//     //   } else {
//     //     rx_ini[i] = q[i];
//     //     printf("Motor %d is SAFE...\n", i + 1);
//     //   }
//     // }
//     // for(int i(0); i < 4; i++) {
//     //   q_home[3 * i] = q[3 * i];
//     //   q_home[3 * i + 1] = q[3 * i + 1] + (pow(-1, i) * 1.65);
//     //   q_home[3 * i + 2] = q[3 * i + 2] + (pow(-1, i + 1) * 1.2);
//     //   printf("Initial Position for leg %d is : %f, %f, %f\n", i, q[3*i], q[3*i + 1], q[3*i + 2]);
//     //   printf("Home Position for leg %d is : %f, %f, %f\n", i, q_home[3*i], q_home[3*i + 1], q_home[3*i + 2]);
//     // }
//     // FirstTimeSafetyCheck = true;
//   }

  

//   // Simulation
//   // kpMat << userParameters.kp, 0, 0, 0,  userParameters.kp, 0, 0, 0,  userParameters.kp;
//   // kdMat << userParameters.kd, 0, 0, 0, userParameters.kd, 0, 0, 0, userParameters.kd;

//   // for(int legIndex(0); legIndex < 4; legIndex++) {
//   //   _legController->commands[legIndex].kpJoint = kpMat;
//   //   _legController->commands[legIndex].kdJoint = kdMat;
//   // }

//   // if(j_iter <= 100) {
//   //   for(int legIndex(0); legIndex < 4; legIndex++) {
//   //     _legController->commands[legIndex].qDes << 0.0, -2.6, 2.5;
//   //     std::cout << _legController->commands[legIndex].qDes << '\n';
//   //   }
//   // } else if (j_iter > 100 && j_iter < 5100){
//   //   for(int i(0); i < 4; i++) {
//   //     q_des[3*i] = 0.0;
//   //     q_des[3*i + 1] = leg_Interpolation(100, j_iter, 5100, -2.6, -0.9);
//   //     q_des[3*i + 2] = 2.5;

//   //     _legController->commands[i].qDes << q_des[3*i], q_des[3*i + 1], q_des[3*i + 2];
//   //   }
//   // } else if (j_iter >=5100 && j_iter < 10100) {
//   //   for(int i(0); i < 4; i++) {
//   //     q_des[3*i] = 0.0;
//   //     q_des[3*i + 1] = -0.9;
//   //     q_des[3*i + 2] = leg_Interpolation(5100, j_iter, 10100, 2.5, 1.5);

//   //     _legController->commands[i].qDes << q_des[3*i], q_des[3*i + 1], q_des[3*i + 2];
//   //   }
//   // } else if (j_iter > 10100) {
//   //   for(int i(0); i < 4; i++) {
//   //     if(leg_enable[i]) {
//   //       updateGamepadCommand();
//   //       _legController->commands[i].qDes << (0.0 + desGamepadCommand[0]), (-0.9 + desGamepadCommand[1]), (1.5 + desGamepadCommand[2]);
//   //     } else {
//   //       _legController->commands[i].qDes << 0.0, -0.9, 1.5;
//   //     }
//   //   }
//   // }

// }

// float Rasp_Controller::leg_Interpolation(const size_t & curr_iter, size_t max_iter,
//                                         const float & ini, const float & fin) {

//     float a(0.f);
//     float b(1.f);

//     // interpolating
//     if(curr_iter <= max_iter) {
//       b = (float)curr_iter / (float)max_iter;
//       a = 1.f - b;
//     }

//     // compute setpoints
//     float inter_pos = a * ini + b * fin;

//     // send control commands
//     return inter_pos;
// }

// void Rasp_Controller::legDetection() {
//   if (_driverCommand->rightTriggerButton) {
//     leg_enable << 0, 1, 0, 0;
//     motion_iter = 0;
//     //printf("Front right leg is enable...\n");
//   }
//   if (_driverCommand->leftTriggerButton) {
//     leg_enable << 1, 0, 0, 0;
//     motion_iter = 0;
//     //printf("Front left leg is enable...\n");
//   }
//   if (_driverCommand->rightBumper) {
//     leg_enable << 0, 0, 0, 1;
//     motion_iter = 0;
//     //printf("Rear right leg is enable...\n");
//   }
//   if (_driverCommand->leftBumper) {
//     leg_enable << 0, 0, 1, 0;
//     motion_iter = 0;
//     //printf("Rear left leg is enable...\n");
//   }
//   if (_driverCommand->x) {
//     leg_enable << 1, 1, 1, 1;
//     motion_iter = 0;
//     currentState = BODY_POS;
//     //printf("All legs are enable...\n");
//   }
//   if (_driverCommand->b) {
//     leg_enable << 0, 0, 0, 0;
//     currentState = GO_HOME;
//     motion_iter = 0;
//     printf("[BREAKING NEWS]: Going Home...\n");
//   }
//   if (_driverCommand->y) {
//     currentState = HI;
//     motion_iter = 0;
//     printf("How you doin...?\n");
//   }
//   if (_driverCommand->a) {
//     currentState = RECOVERY_STAND;
//     motion_iter = 0;
//     printf("[BREAKING NEWS]: Back to stand up..\n");
//   }
// }

// void Rasp_Controller::updateGamepadCommand() {
//   move_range = userParameters.move_range;
//   desGamepadCommand[0] = _driverCommand->leftStickAnalog[0] * move_range;
//   desGamepadCommand[1] = _driverCommand->leftStickAnalog[1] * move_range;
//   desGamepadCommand[2] = _driverCommand->rightStickAnalog[1] * move_range;
//   //printf("Gamepad Command: %f, %f, %f\n", desGamepadCommand[0], desGamepadCommand[1], desGamepadCommand[2]);
// }