/*============================ Control FSM ============================*/
/**
 * The Finite State Machine that manages the robot's controls. Handles
 * calls to the FSM State functions and manages transitions between all
 * of the states.
 */

#include "ControlFSM.h"
#include <rt/rt_rc_interface.h>
#include <chrono>
/**
 * Constructor for the Control FSM. Passes in all of the necessary
 * data and stores it in a struct. Initializes the FSM with a starting
 * state and operating mode.
 *
 * @param _quadruped the quadruped information
 * @param _stateEstimator contains the estimated states
 * @param _legController interface to the leg controllers
 * @param _gaitScheduler controls scheduled foot contact modes
 * @param _desiredStateCommand gets the desired COM state trajectories
 * @param controlParameters passes in the control parameters from the GUI
 */
template <typename T>
ControlFSM<T>::ControlFSM(Quadruped<T>* _quadruped,
                          StateEstimatorContainer<T>* _stateEstimator,
                          LegController<T>* _legController,
                          GaitScheduler<T>* _gaitScheduler,
                          DesiredStateCommand<T>* _desiredStateCommand,
                          RobotControlParameters* controlParameters,
                          VisualizationData* visualizationData,
                          MiLAB_UserParameters* userParameters)
{
  // Add the pointers to the ControlFSMData struct
  data._quadruped = _quadruped;
  data._stateEstimator = _stateEstimator;
  data._legController = _legController;
  data._gaitScheduler = _gaitScheduler;
  data._desiredStateCommand = _desiredStateCommand;
  data.controlParameters = controlParameters;
  data.visualizationData = visualizationData;
  data.userParameters = userParameters;

  // Initialize and add all of the FSM States to the state list
  statesList.invalid = nullptr;
  statesList.passive = new FSM_State_Passive<T>(&data);
  statesList.jointPD = new FSM_State_JointPD<T>(&data);
  statesList.impedanceControl = new FSM_State_ImpedanceControl<T>(&data);
  statesList.standUp = new FSM_State_StandUp<T>(&data);
  statesList.balanceStand = new FSM_State_BalanceStand<T>(&data);
  statesList.locomotion = new FSM_State_Locomotion<T>(&data);
  statesList.recoveryStand = new FSM_State_RecoveryStand<T>(&data);
  statesList.vision = new FSM_State_Vision<T>(&data);
  statesList.backflip = new FSM_State_BackFlip<T>(&data);
  statesList.frontJump = new FSM_State_FrontJump<T>(&data);
  statesList.squatDown = new FSM_State_SquatDown<T>(&data);
  safetyChecker = new SafetyChecker<T>(&data);

  lcm = new lcm::LCM("udpm://239.255.76.67:7667?ttl=1");
  if (!lcm->good()) {
    std::cerr << "LCM initialization failed" << std::endl;
    exit(1);  // Exit if initialization fails
  }
  lcm->subscribe("interface", &ControlFSM::handleGamepadData, this);



  // Initialize the FSM with the Passive FSM State
  initialize();
}
template <typename T>
void ControlFSM<T>::handleGamepadData(const lcm::ReceiveBuffer* rbuf,
                                      const std::string& chan,
                                      const gamepad_lcmt* msg) {
  (void)rbuf;
  (void)chan;
  gamepadCommand = *msg; // Update the gamepad command with the received data
}


/**
 * Runs the LCM handling to continuously process joystick input.
 */
template <typename T>
void ControlFSM<T>::handleLCM() {
  lcm->handleTimeout(0.1);
  // while (0 == lcm->handleTimeout(10)) {
  //   // Continuously handle incoming LCM messages
  // }
}

auto current_time(){
  return std::chrono::high_resolution_clock::now();
}

auto time_diff(const std::chrono::time_point<std::chrono::high_resolution_clock>& start) {
  return std::chrono::duration_cast<std::chrono::milliseconds>(current_time() - start).count();
  //std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(current_time() - start).count() << std::endl;
  //std::cout << std::chrono::high_resolution_clock::now() << std::endl;
}
auto start_time = current_time();
/**
 * Initialize the Control FSM with the default settings. SHould be set to
 * Passive state and Normal operation mode.
 */
template <typename T>
void ControlFSM<T>::initialize() {
  // Initialize a new FSM State with the control data
  currentState = statesList.passive;

  // Enter the new current state cleanly
  currentState->onEnter();

  // Initialize to not be in transition
  nextState = currentState;

  // Initialize FSM mode to normal operation
  operatingMode = FSM_OperatingMode::NORMAL;
}

/**
 * Called each control loop iteration. Decides if the robot is safe to
 * run controls and checks the current state for any transitions. Runs
 * the regular state behavior if all is normal.
 */
template <typename T>
void ControlFSM<T>::runFSM() {
  
  // Publish state estimator data to other computer
  //for(size_t i(0); i<3; ++i){
    //_state_estimator.p[i] = data._stateEstimator->getResult().position[i];
    //_state_estimator.quat[i] = data._stateEstimator->getResult().orientation[i];
  //}
    //_state_estimator.quat[3] = data._stateEstimator->getResult().orientation[3];
  //state_estimator_lcm.publish("state_estimator_ctrl_pc", &_state_estimator);

  // Check the robot state for safe operation
  operatingMode = safetyPreCheck();

  handleLCM();


  if(data.controlParameters->use_rc){
    int rc_mode = data._desiredStateCommand->rcCommand->mode;
    if(rc_mode == RC_mode::OFF){
          printf("K_PASSIVE\n");
          data.controlParameters->control_mode = K_PASSIVE;
    }else if(rc_mode == RC_mode::RECOVERY_STAND){
      data.controlParameters->control_mode = K_RECOVERY_STAND;

    } else if(rc_mode == RC_mode::LOCOMOTION){
      data.controlParameters->control_mode = K_LOCOMOTION;

    } else if(rc_mode == RC_mode::QP_STAND){
      data.controlParameters->control_mode = K_BALANCE_STAND;

    } else if(rc_mode == RC_mode::SQUAT_DOWN){
        data.controlParameters->control_mode = K_SQUAT_DOWN;

    } else if (rc_mode == RC_mode::STAND_UP){
        data.controlParameters->control_mode = K_STAND_UP;
    }

  }
  //bool recoverydata = &recoverycommand->a;
  //lcm_gamepad.subscribe("gamepad_command", &gamepad_lcm)
  
  if (false)
  {
    // if (time_diff(start_time)>=10000)
    // // if (data._desiredStateCommand->gamepadCommand->a || recoverymode)
    // {
      
    //   std::cout<<time_diff(start_time)<<std::endl;
    //   recoverymode = true;
    //   data.controlParameters->control_mode = K_RECOVERY_STAND;
    //   if (false)
    //   {
    //     recoverymode = false;
    //   }
      
      
    //   // printf("[Recovery Balance]recoveeeeeeeeeeeeeeeeeeeeeeeeery\n");
    // }
    // else if (false)
    // {
    //   recoverymode = false;
    //   squatmode = true;
    //   data.controlParameters->control_mode = K_PASSIVE;
    //   printf("[Recovery Balance]squaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaat \n");
    // }
    // else
    // {
    //   data.controlParameters->control_mode = K_PASSIVE;
    //   recoverymode = false;
    //   printf("PAAAAAsiiiiiiiiiiiiiv\n");
    // }
      if ((gamepadCommand.a || recoverymode) && !(gamepadCommand.b || gamepadCommand.x || gamepadCommand.y)) {
    recoverymode = true;
    squatmode = false;
    standup_mode = false;
    data.controlParameters->control_mode = K_RECOVERY_STAND;
    std::cout << "Gamepad button A pressed: Switching to Recovery Stand mode." << std::endl;

} else if ((gamepadCommand.b || squatmode) && !(gamepadCommand.a || gamepadCommand.x || gamepadCommand.y)) {
    squatmode = true;
    recoverymode = false;
    standup_mode = false;
    data.controlParameters->control_mode = K_SQUAT_DOWN;
    std::cout << "Gamepad button B pressed: Switching to Squat Down mode." << std::endl;

} else if ((gamepadCommand.x || standup_mode) && !(gamepadCommand.a || gamepadCommand.b || gamepadCommand.y)) {
    standup_mode = true;
    recoverymode = false;
    squatmode = false;
    data.controlParameters->control_mode = K_STAND_UP;
    std::cout << "Gamepad button X pressed: Switching to Stand Up mode." << std::endl;

} 
else {
    standup_mode = false;
    recoverymode = false;
    squatmode = false;
    data.controlParameters->control_mode = K_PASSIVE;
    std::cout << "No significant gamepad input: Remaining in Passive mode." << std::endl;
}
    
    
  }
  // printf("OUUUUUUUUUUUUT\n");
  
  //std::cout<< recoverydata <<std::endl;
  //std::cout<< _driverCommand->a <<std::endl;

  // Run the robot control code if operating mode is not unsafe
  if (operatingMode != FSM_OperatingMode::ESTOP) {
    // Run normal controls if no transition is detected
    if (operatingMode == FSM_OperatingMode::NORMAL) {
      // Check the current state for any transition
      nextStateName = currentState->checkTransition();

      // Detect a commanded transition
      if (nextStateName != currentState->stateName) {
        // Set the FSM operating mode to transitioning
        operatingMode = FSM_OperatingMode::TRANSITIONING;

        // Get the next FSM State by name
        nextState = getNextState(nextStateName);

        // Print transition initialized info
        //printInfo(1);

      } else {
        // Run the iteration for the current state normally
        currentState->run();
      }
    }

    // Run the transition code while transition is occuring
    if (operatingMode == FSM_OperatingMode::TRANSITIONING) {
      transitionData = currentState->transition();

      // Check the robot state for safe operation
      safetyPostCheck();

      // Run the state transition
      if (transitionData.done) {
        // Exit the current state cleanly
        currentState->onExit();

        // Print finalizing transition info
        //printInfo(2);

        // Complete the transition
        currentState = nextState;

        // Enter the new current state cleanly
        currentState->onEnter();

        // Return the FSM to normal operation mode
        operatingMode = FSM_OperatingMode::NORMAL;
      }
    } else {
      // Check the robot state for safe operation
      safetyPostCheck();
    }

  } else { // if ESTOP
    currentState = statesList.passive;
    currentState->onEnter();
    nextStateName = currentState->stateName;
  }

  // Print the current state of the FSM
  printInfo(0);

  // Increase the iteration counter
  iter++;
}

/**
 * Checks the robot state for safe operation conditions. If it is in
 * an unsafe state, it will not run the normal control code until it
 * is safe to operate again.
 *
 * @return the appropriate operating mode
 */
template <typename T>
FSM_OperatingMode ControlFSM<T>::safetyPreCheck() {
  // Check for safe orientation if the current state requires it
  if (currentState->checkSafeOrientation && data.controlParameters->control_mode != K_RECOVERY_STAND) {
    if (!safetyChecker->checkSafeOrientation()) {
      operatingMode = FSM_OperatingMode::ESTOP;
      std::cout << "broken: Orientation Safety Ceck FAIL" << std::endl;
    }
  }

  // Default is to return the current operating mode
  return operatingMode;
}

/**
 * Checks the robot state for safe operation commands after calculating the
 * control iteration. Prints out which command is unsafe. Each state has
 * the option to enable checks for commands that it cares about.
 *
 * Should this EDamp / EStop or just continue?
 * Should break each separate check into its own function for clarity
 *
 * @return the appropriate operating mode
 */
template <typename T>
FSM_OperatingMode ControlFSM<T>::safetyPostCheck() {
  // Check for safe desired foot positions
  if (currentState->checkPDesFoot) {
    safetyChecker->checkPDesFoot();
  }

  // Check for safe desired feedforward forces
  if (currentState->checkForceFeedForward) {
    safetyChecker->checkForceFeedForward();
  }

  // Default is to return the current operating mode
  return operatingMode;
}

/**
 * Returns the approptiate next FSM State when commanded.
 *
 * @param  next commanded enumerated state name
 * @return next FSM state
 */
template <typename T>
FSM_State<T>* ControlFSM<T>::getNextState(FSM_StateName stateName) {
  // Choose the correct FSM State by enumerated state name
  switch (stateName) {
    case FSM_StateName::INVALID:
      return statesList.invalid;

    case FSM_StateName::PASSIVE:
      return statesList.passive;

    case FSM_StateName::JOINT_PD:
      return statesList.jointPD;

    case FSM_StateName::IMPEDANCE_CONTROL:
      return statesList.impedanceControl;

    case FSM_StateName::STAND_UP:
      return statesList.standUp;

    case FSM_StateName::BALANCE_STAND:
      return statesList.balanceStand;

    case FSM_StateName::LOCOMOTION:
      return statesList.locomotion;

    case FSM_StateName::RECOVERY_STAND:
      return statesList.recoveryStand;

    case FSM_StateName::VISION:
      return statesList.vision;

    case FSM_StateName::BACKFLIP:
      return statesList.backflip;

    case FSM_StateName::FRONTJUMP:
      return statesList.frontJump;

      case FSM_StateName::SQUAT_DOWN:
          return statesList.squatDown;

    default:
      return statesList.invalid;
  }
}

/**
 * Prints Control FSM info at regular intervals and on important events
 * such as transition initializations and finalizations. Separate function
 * to not clutter the actual code.
 *
 * @param printing mode option for regular or an event
 */
template <typename T>
void ControlFSM<T>::printInfo(int opt) {
  switch (opt) {
    case 0:  // Normal printing case at regular intervals
      // Increment printing iteration
      printIter++;

      printNum = 200;

      // Print at commanded frequency
      if (true) {
        std::cout << "[CONTROL FSM] Printing FSM Info...\n";
        std::cout
            << "---------------------------------------------------------\n";

             // Get state estimation data
            auto orientation_test = data._stateEstimator->getResult().orientation;
            auto orientation_rpy_test = data._stateEstimator->getResult().rpy*180/M_PI;
            auto position_test = data._stateEstimator->getResult().position;
            auto linearVelocity_test = data._stateEstimator->getResult().vBody;
            auto rotationalVelocity_test = data._stateEstimator->getResult().omegaBody;
            auto acc_test = data._stateEstimator->getResult().aBody;


            // Write data 
            std::cout << "Quaternion: " << orientation_test[0] << "," << orientation_test[1] << "," << orientation_test[2] << "," << orientation_test[3] << std::endl
                      << "RPY: " << orientation_rpy_test[0] << "," << orientation_rpy_test[1] << "," << orientation_rpy_test[2] << std::endl
                      << "Position: " << position_test[0] << "," << position_test[1] << "," << position_test[2] << std::endl
                      << "Velocity: " << linearVelocity_test[0] << "," << linearVelocity_test[1] << "," << linearVelocity_test[2] << std::endl
                      << "Omega: " << rotationalVelocity_test[0] << "," << rotationalVelocity_test[1] << "," << rotationalVelocity_test[2] << std::endl 
                      << "ACC Body: " << acc_test[0] << "," << acc_test[1] << "," << acc_test[2] << std::endl;


        std::cout << "Iteration: " << iter << "\n";
        if (operatingMode == FSM_OperatingMode::NORMAL) {
          std::cout << "Operating Mode: NORMAL in " << currentState->stateString
                    << "\n";

        } else if (operatingMode == FSM_OperatingMode::TRANSITIONING) {
          std::cout << "Operating Mode: TRANSITIONING from "
                    << currentState->stateString << " to "
                    << nextState->stateString << "\n";

        } else if (operatingMode == FSM_OperatingMode::ESTOP) {
          std::cout << "Operating Mode: ESTOP\n";
        }
        std::cout << "Gait Type: " << data.userParameters->cmpc_gait
                  << "\n";
        std::cout << std::endl;

        // Reset iteration counter
        printIter = 0;
      }

      // Print robot info about the robot's status
      // data._gaitScheduler->printGaitInfo();
      // data._desiredStateCommand->printStateCommandInfo();

      break;

    case 1:  // Initializing FSM State transition
      std::cout << "[CONTROL FSM] Transition initialized from "
                << currentState->stateString << " to " << nextState->stateString
                << "\n"
                << std::endl;

      break;

    case 2:  // Finalizing FSM State transition
      std::cout << "[CONTROL FSM] Transition finalizing from "
                << currentState->stateString << " to " << nextState->stateString
                << "\n"
                << std::endl;

      break;
  }
}
// #include <fstream> // Include this for file handling

// void ControlFSM<T>::printInfo(int opt) {
//   static std::ofstream logFile("FSM_Logs.txt", std::ios::app); // Append to the file
//   if (!logFile.is_open()) {
//     std::cerr << "Error: Unable to open log file!\n";
//     return;
//   }

//   switch (opt) {
//     case 0:  // Normal printing case at regular intervals
//       printIter++;
//       printNum = 200;

//       if (true) { // Change this to true to enable logging
//         // logFile << "[CONTROL FSM] Printing FSM Info...\n";
//         // logFile << "---------------------------------------------------------\n";

//         auto orientation_test = data._stateEstimator->getResult().orientation;
//         auto orientation_rpy_test = data._stateEstimator->getResult().rpy * 180 / M_PI;
//         auto position_test = data._stateEstimator->getResult().position;
//         auto linearVelocity_test = data._stateEstimator->getResult().vBody;
//         auto rotationalVelocity_test = data._stateEstimator->getResult().omegaBody;

//         logFile << "Quaternion: " << orientation_test[0] << ", " << orientation_test[1] << ", " 
//                 << orientation_test[2] << ", " << orientation_test[3] << "\n"
//                 << "RPY: " << orientation_rpy_test[0] << ", " << orientation_rpy_test[1] << ", " 
//                 << orientation_rpy_test[2] << "\n"
//                 << "Position: " << position_test[0] << ", " << position_test[1] << ", " 
//                 << position_test[2] << "\n"
//                 << "Velocity: " << linearVelocity_test[0] << ", " << linearVelocity_test[1] << ", " 
//                 << linearVelocity_test[2] << "\n"
//                 << "Omega: " << rotationalVelocity_test[0] << ", " << rotationalVelocity_test[1] 
//                 << ", " << rotationalVelocity_test[2] << "\n";

//         logFile << "Iteration: " << iter << "\n";
//         // if (operatingMode == FSM_OperatingMode::NORMAL) {
//         //   logFile << "Operating Mode: NORMAL in " << currentState->stateString << "\n";
//         // } else if (operatingMode == FSM_OperatingMode::TRANSITIONING) {
//         //   logFile << "Operating Mode: TRANSITIONING from " << currentState->stateString
//         //           << " to " << nextState->stateString << "\n";
//         // } else if (operatingMode == FSM_OperatingMode::ESTOP) {
//         //   logFile << "Operating Mode: ESTOP\n";
//         // }
//         // logFile << "Gait Type: " << data.userParameters->cmpc_gait << "\n\n";

//         printIter = 0;
//       }
//       break;

//     case 1:  // Initializing FSM State transition
//       logFile << "[CONTROL FSM] Transition initialized from "
//               << currentState->stateString << " to " << nextState->stateString
//               << "\n\n";
//       break;

//     case 2:  // Finalizing FSM State transition
//       logFile << "[CONTROL FSM] Transition finalizing from "
//               << currentState->stateString << " to " << nextState->stateString
//               << "\n\n";
//       break;
//   }
// }


// template class ControlFSM<double>; This should be fixed... need to make
// RobotRunner a template
template class ControlFSM<float>;
