#include "New_Controller.hpp"

void New_Controller::legDetection(){

    if (_driverCommand->rightTriggerButton){
        leg_enable << 1, 0, 0, 0;
        printf("Front right leg is enable...\n");
    }
    if (_driverCommand->leftTriggerButton){
        leg_enable << 0, 1, 0, 0;
        printf("Front left leg is enable...\n");
    }
    if (_driverCommand->rightBumper){
        leg_enable << 0, 0, 1, 0;
        printf("Rear right leg is enable...\n");
    }
    if (_driverCommand->leftBumper){
        leg_enable << 0, 0, 0, 1;
        printf("Rear left leg is enable...\n");
    }
    if (_driverCommand->x){
        leg_enable << 1, 1, 1, 1;
        printf("All legs are enable...\n");
    }
}

void New_Controller::jointDetection(){
    if (_driverCommand->a){
        joint_enable << 0, 0, 1;
        printf("Knee joint is enable...\n");
    }
    if (_driverCommand->b){
        joint_enable << 0, 1, 0;
        printf("Hip joint is enable...\n");
    }
    if (_driverCommand->y){
        joint_enable << 1, 0, 0;
        printf("Ab/Ad joint is enable...\n");
    }
}


void New_Controller::initializeController(){

    _legController->_maxTorque = userParameters.max_tau;
    _legController->_legsEnabled = true;

    joint_enable << 0, 0, 0;
    leg_enable << 0, 0, 0, 0;

    if(userParameters.calibrate > 0.4) {  // This param is only used in TI board in Cheetah3
        _legController->_calibrateEncoders = userParameters.calibrate;
    } else {
        if (userParameters.zero > 0.5) {  //　This param is only used in TI board in Cheetah3
            _legController->_zeroEncoders = true;
        } else {
            _legController->_zeroEncoders = false;
        }
    }
    for (int leg = 0; leg < 4; ++leg) {
        _new_fold[leg] << 0.f, -1.4f, 2.7f;
        _new_stand[leg] << 0.f, -0.8f, 1.6f;
    }
}

void New_Controller::runController(){

    legDetection();
    jointDetection();

    //Mat3 <float> r ;//= _stateEstimate->rBody;
    //r << 1,0,0,0,1,0,0,0,1;
    //Vec3 <float> p = _stateEstimate->position;
    //Vec3 <float> v = _stateEstimate->vWorld;

//    Joint pd
    kpMat << userParameters.kpj[0], 0, 0, 0,  userParameters.kpj[1], 0, 0, 0,  userParameters.kpj[2];
    kdMat << userParameters.kdj[0], 0, 0, 0, userParameters.kdj[1], 0, 0, 0, userParameters.kdj[2];
//    Cartesian pd
    kpcMat << userParameters.kpc[0], 0, 0, 0,  userParameters.kpc[1], 0, 0, 0,  userParameters.kpc[2];
    kdcMat << userParameters.kdc[0], 0, 0, 0, userParameters.kdc[1], 0, 0, 0, userParameters.kdc[2];

    //Vec3 <float> phip = _quadruped->getHipLocation(0);

    // Get robot initial joint position
    if(j_iter < 100){
        for(int leg(0); leg<4; ++leg){
            for(int jidx(0); jidx<3; ++jidx){
                _new_ini[leg][jidx] = _legController->datas[leg].q[jidx];
            }
        }
    }
//    std::cout<<_jpos_ini[0]<<std::endl;
    if (j_iter%5000 == 0 && j_iter < userParameters.max_iter)
        printf("[Jpos_Ctrl] INFO: Control iteration is %ld now ...\n\n", j_iter);

    if(userParameters.calibrate <= 0.4 && !_legController->_zeroEncoders) {
        motion_iter++;

//      fold legs
        if (j_iter>= 100 && flag == FOLDLEG){
            for (int leg = 0; leg < 4; ++leg) {
                if (motion_iter >= fold_iter+wait_iter){
                    motion_iter =0;
                    flag = STANDUP;
                }
                else if (motion_iter > fold_iter && motion_iter < fold_iter+wait_iter)
                    leg_JointPD(leg, _new_fold[leg], zero_vec3);
                else
                    leg_Interpolation(motion_iter,fold_iter,leg,_new_ini[leg],_new_fold[leg]);
            }

//            stand up and keep standing
        } else if (flag == STANDUP){
            for (int leg = 0; leg < 4; ++leg) {
                if (motion_iter >= stand_iter+keep_iter){
                    motion_iter =0;
                    flag = JOINT;
                    //_pfoot_ini = p + r.transpose() * (phip +_legController->datas[0].p);
                }
                else if (motion_iter > stand_iter && motion_iter < stand_iter+keep_iter)
                    leg_JointPD(leg, _new_stand[leg], zero_vec3);
                else
                    leg_Interpolation(motion_iter,stand_iter,leg,_new_fold[leg],_new_stand[leg]);
            }
//            squad down and motor switch down
        } else if (flag == JOINT) {
            for(int leg(0); leg<4; ++leg){
                _legController->commands[leg].kpJoint = kpMat;
                _legController->commands[leg].kdJoint = kdMat;
                if (leg_enable[leg]){

                    _legController->commands[leg].qDes[0] = _new_stand[leg][0] + _driverCommand->leftStickAnalog[1];
                    _legController->commands[leg].qDes[1] = _new_stand[leg][1] + _driverCommand->leftStickAnalog[0];
                    _legController->commands[leg].qDes[2] = _new_stand[leg][2] + _driverCommand->rightStickAnalog[1];

                    // for(int jidx(0); jidx<3; ++jidx){
                    //     if (joint_enable[jidx]){
                    //         Eigen::Vector2f gamepadcmd = _driverCommand->leftStickAnalog;
                    //         printf("Desired Velocity is %f\n",gamepadcmd[1]);
                    //         if (jidx == 2)
                    //             _legController->commands[leg].qDes[jidx] = _new_stand[leg][jidx] + gamepadcmd[1] * userParameters.move_range * 6;
                    //         else
                    //             _legController->commands[leg].qDes[jidx] = _new_stand[leg][jidx] + gamepadcmd[1] * userParameters.move_range;
                    //     } else {
                    //         _legController->commands[leg].qDes[jidx] = _legController->datas[leg].q[jidx];
                    //     }
                    // }
                } else {
                    for(int jidx(0); jidx<3; ++jidx) {
                        _legController->commands[leg].qDes[jidx] = _new_stand[leg][jidx];
                    }
                }
            }
        }
    }
    ++j_iter;
}

void New_Controller::leg_Interpolation(const size_t & curr_iter, size_t max_iter, int leg,
                                        const Vec3<float> & ini, const Vec3<float> & fin){

    float a(0.f);
    float b(1.f);

    // interpolating
    if(curr_iter <= max_iter) {
      b = (float)curr_iter/(float)max_iter;
      a = 1.f - b;
    }

    // compute setpoints
    Vec3<float> inter_pos = a * ini + b * fin;

    // send control commands
    leg_JointPD(leg, inter_pos, zero_vec3);
}
void New_Controller::leg_JointPD(int leg, Vec3<float> qDes, Vec3<float> qdDes) {

    _legController->commands[leg].kpJoint = kpMat;
    _legController->commands[leg].kdJoint = kdMat;

    _legController->commands[leg].qDes = qDes;
    _legController->commands[leg].qdDes = qdDes;
}

void New_Controller::leg_Passive(int leg){

    _legController->commands[leg].kpJoint.setZero();
    _legController->commands[leg].kdJoint.setZero();
    for (int jidx = 0; jidx < 3; ++jidx) {
        _legController->commands[leg].tauFeedForward[jidx] = 0.;
    }

}