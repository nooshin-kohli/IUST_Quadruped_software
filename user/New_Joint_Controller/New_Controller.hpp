#ifndef NEW_CONTROLLER
#define NEW_CONTROLLER

#include <RobotController.h>
#include "NewUserParameters.h"
#include <Controllers/FootSwingTrajectory.h>
#include <Controllers/StateEstimatorContainer.h>
#include <Dynamics/Quadruped.h>
#define NEW_CARTESIAN

class New_Controller:public RobotController{
  public:
    New_Controller():RobotController(){ }
    void leg_Interpolation(const size_t & curr_iter, size_t max_iter, int leg,
                           const Vec3<float> & ini, const Vec3<float> & fin);
    void leg_JointPD(int leg, Vec3<float> qDes, Vec3<float> qdDes);
    void leg_Passive(int leg);
    void legDetection();
    void jointDetection();

    virtual ~New_Controller(){}
    virtual void initializeController();
    virtual void runController();
    virtual void updateVisualization(){}
    virtual ControlParameters* getUserControlParameters() {
      return &userParameters;
    }
  protected:
    Vec3<float> _new_ini[4];
    Vec3<float> _new_fold[4];
    Vec3<float> _new_stand[4];
    Vec3<float> _pfoot_ini;
    Vec3<float> zero_vec3;
    NewUserParameters userParameters;
    Vec3<int> joint_enable;
    Vec4<int> leg_enable;

  private:
    StateEstimatorContainer<float>* _stateEstimator;
    Mat3<float> kpMat;
    Mat3<float> kdMat;
    Mat3<float> kpcMat;
    Mat3<float> kdcMat;
    FootSwingTrajectory<float> footSwingTrajectories[4];
    const int fold_iter = 400;
    const int stand_iter = 300;
    const int squad_iter = 500;
    const int wait_iter = 500;
    const int keep_iter = 1000;
    const int cart_iter = 2000;
    static const int FOLDLEG =0;
    static const int STANDUP =1;
    static const int SQUAD =2;
    static const int JOINT =3;
    int flag = FOLDLEG;
    long j_iter=0;
    int motion_iter=0;
};

#endif