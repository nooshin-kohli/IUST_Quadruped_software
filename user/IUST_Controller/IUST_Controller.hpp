#ifndef JPOS_CONTROLLER
#define JPOS_CONTROLLER

#include <RobotController.h>
#include "IustUserParameters.h"
#include <RobotController.h>
#include <RobotRunner.h>
class IUST_Controller:public RobotController{
  public:
    IUST_Controller():RobotController(),_jpos_ini(cheetah::num_act_joint){
    _jpos_ini.setZero();
    }
    virtual ~IUST_Controller(){}

    virtual void initializeController();
    virtual void runController();
    virtual void updateVisualization(){};
    // virtual void safety_check(float q_current[12]);
    virtual void calculateQHome(const float q_initial[12]);
    virtual void checkMaxPostureExceeded(float position[12], float Q_home[12]);
    float leg_Interpolation(const size_t & curr_iter, size_t max_iter,
                           const float & ini, const float & fin);
    virtual void Open_Calf_Joint(const size_t & curr_iter);   
    virtual void Standup(const size_t & curr_iter);                    
    virtual ControlParameters* getUserControlParameters() {
      return &userParameters;
    }

  private:
    int debug_iter;
    bool first_time;
    bool safety=true;
    float q_now[12], q_des[12],q_ini_resp[12],q_ini_knee[12],q_ini_hip[12],q_home[12];


    // this is for motor data:
    // float q_ini[12] = {0.0088f, -0.3332f, 0.0146f,
    //                    -0.1305f, -0.2927f, -0.1738f,
    //                    -0.7014f, -0.3941f, -0.9566f,
    //                    0.0030f, -0.7971f, -0.7420f};


  float q_ini[12] = {-0.0289f, 2.7368f, -2.5386f,
                       0.0087f, 2.6527f, -2.4025f,
                       -0.0550f, 2.6672f, -2.6057f,
                       0.0318f, 2.6904f, -2.5386f};

  protected:
    DVec<float> _jpos_ini;
    IustUserParameters userParameters;
};

#endif
