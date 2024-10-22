// #ifndef JPOS_CONTROLLER
// #define JPOS_CONTROLLER

// #define UNUSED(x) (void)(x)


// #include <RobotController.h>
// //#include <RobotRunner.h>
// #include "RaspberryPiParameters.h"
// #include <Controllers/FootSwingTrajectory.h>
// #include <Controllers/StateEstimatorContainer.h>
// #include <Dynamics/Quadruped.h>

// #include <unistd.h>
// #include <lcm/lcm-cpp.hpp>
// #include "leg_control_command_lcmt.hpp"
// #include "leg_control_data_lcmt.hpp"

// extern lcm::LCM rasp_lcm;

// extern float q[12];
// extern float qd[12];
// extern float p[12];
// extern float v[12];
// extern float tau_est[12];
// extern bool safetyCheck;

// class Rasp_Controller:public RobotController{
//   public:
//     Rasp_Controller():RobotController(){ }

//     virtual void initializeController();
//     virtual void runController();
//     virtual void updateVisualization(){ }
//     void updateRaspCommand();
//     void updateGamepadCommand();
//     void legDetection();
//     float leg_Interpolation(const size_t & curr_iter, size_t max_iter,
//                            const float & ini, const float & fin);
//     virtual ControlParameters* getUserControlParameters() {
//       return &userParameters;
//     }
//   protected:
//     float rx_ini[12], q_home[12], q_now[12];
//     float q_ini[12] = {-0.6435f, -0.3187f, -0.6927f,
//                        -0.7594f, -0.2840f, -0.1709f,
//                        -0.8087f, -0.4086f, -0.9566f,
//                        -0.5681f,  0.0146f -1, -0.7188f};
//     Vec3<float> _pfoot_ini;
//     Vec3<float> zero_vec3;
//     RaspUserParameters userParameters;
//     Mat3<float> kpMat;
//     Mat3<float> kdMat;

//   private:
//     StateEstimatorContainer<float>* _stateEstimator;

//     leg_control_command_lcmt COMMAND;

//     static const int FOLDLEG = 0;
//     static const int HIP_HOMING = 1;
//     static const int KNEE_HOMING = 2;
//     static const int JOINT = 3;
//     static const int GO_HOME = 4;
//     static const int SHUTDOWN = 5;
//     static const int HI = 6;
//     static const int RECOVERY_STAND = 7;
//     static const int BODY_POS = 8;
//     int m = 0;
//     int currentState = FOLDLEG;

//     const int wait_iter = 600;
//     const int hip_iter = 300;
//     const int knee_iter = 300;
//     //
//     const int shutdown_iter = 600;
//     const int keep_iter = 200;
//     const int recovery_iter = 300;
//     long j_iter = 0;
//     int motion_iter = 0;

//     float move_range;
//     bool SetHomePosition = false;
//     bool FirstTimeSafetyCheck = false;
//     bool yekBar = true;
//     Vec4<int> leg_enable;
//     Vec3<float> desGamepadCommand;

// };

// // class Handler {
// //     public:
// //         void handleMessage(const lcm::ReceiveBuffer* rbuf,
// //                 const std::string& chan,
// //                 const leg_control_data_lcmt* msg)
// //         {
// //           UNUSED(rbuf);
// //           UNUSED(chan);
// //           //printf("Receiving message on channel \"%s\":\n", chan.c_str());
// //           for(int i(0); i < 12; i++) {
// //             if(std::abs(q[i] - msg->q[i] > 0.1)) {
// //               // safetyCheck = false;
// //               while (true)
// //               {
// //                 printf("[WARNING]: Command for motor %d is not safe ----> q: %f   ,   q_des: %f\n", i + 1, q[i], msg->q[i]);
// //               }
// //             } else {
// //               q[i] = msg->q[i];
// //               qd[i] = msg->qd[i];
// //               p[i] = msg->p[i];
// //               v[i] = msg->v[i];
// //               tau_est[i] = msg->tau_est[i];
// //               //printf("Data for motor number %d : %f , %f \n", i, q[i], qd[i]);
// //             }
// //           }  
// //         }
// // };

// #endif