/*!
 * @file HardwareBridge.h
 * @brief Interface between robot code and robot hardware
 *
 * This class initializes the hardware of both robots and allows the robot
 * controller to access it. It is specialized for Linux systems.
 */

#ifndef PROJECT_HARDWAREBRIDGE_H
#define PROJECT_HARDWAREBRIDGE_H

#ifdef linux 

// Maximum stack size for the robot
#define MAX_STACK_SIZE 16384  // 16KB  of stack
// Linux priority, this is not the nice value
#define TASK_PRIORITY 49

#include <string>
#include <lcm/lcm-cpp.hpp>
#include <lord_imu/LordImu.h>

#include "RobotRunner.h"
#include "Utilities/PeriodicTask.h"
#include "control_parameter_request_lcmt.hpp"
#include "control_parameter_respones_lcmt.hpp"
#include "actuator_command_t.hpp"
#include "actuator_response_t.hpp"
#include "gamepad_lcmt.hpp"
#include "microstrain_lcmt.hpp"
#include "ecat_command_t.hpp"
#include "ecat_data_t.hpp"
#include "rt/rt_can.h"


/*!
 * Interface between robot and hardware
 */
class HardwareBridge {
 public:
 /*!
   * Constructor for the HardwareBridge class.
   * 
   * @param robot_ctrl: Pointer to the RobotController object.
   */
  HardwareBridge(RobotController* robot_ctrl)
                : statusTask(&taskManager, 0.5f),
                _interfaceLCM(getLcmUrl(255)),
                _visualizationLCM(getLcmUrl(255)) {
                  _controller = robot_ctrl;
                  _userControlParameters = robot_ctrl->getUserControlParameters();
                }
  /*!
   * This function is used to prefault the stack.
   */
  void prefaultStack();
  /*!
   * This function is used to setup the scheduler.
   */
  void setupScheduler();
  /*!
   * This function is used to initialize the error.
   * 
   * @param reason: The reason for the error.
   * @param printErrno: Boolean value to indicate if the errno should be printed.
   */
  void initError(const char* reason, bool printErrno = false);
  /*!
   * This function is used to initialize the common elements.
   */
  void initCommon();
  /*!
   * Destructor for the HardwareBridge class.
   */
  ~HardwareBridge() {
    delete _robotRunner;
  }
  /*!
   * This function is used to handle the gamepad LCM.
   * 
   * @param rbuf: The receive buffer.
   * @param chan: The channel.
   * @param msg: The message.
   */
  void handleGamepadLCM(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                        const gamepad_lcmt* msg);
  /*!
   * This function is used to handle the interface LCM.
   */
  void handleInterfaceLCM();
  /*!
   * This function is used to handle the control parameter.
   * 
   * @param rbuf: The receive buffer.
   * @param chan: The channel.
   * @param msg: The message.
   */
  void handleControlParameter(const lcm::ReceiveBuffer* rbuf,
                              const std::string& chan,
                              const control_parameter_request_lcmt* msg);
  /*!
   * This function is used to publish the visualization LCM.
   */
  void publishVisualizationLCM();
  /*!
   * This function is used to run the sbus.
   */
  void run_sbus();

 protected:
  PeriodicTaskManager taskManager;
  PrintTaskStatus statusTask;
  GamepadCommand _gamepadCommand;
  VisualizationData _visualizationData;
  CheetahVisualization _mainCheetahVisualization;
  lcm::LCM _interfaceLCM;
  lcm::LCM _visualizationLCM;
  control_parameter_respones_lcmt _parameter_response_lcmt;
  CANData _canData;
  CANCommand _canCommand;
  SpiCommand _spiCommand;
  SpiData _spiData;

  TiBoardCommand _tiBoardCommand[4];
  TiBoardData _tiBoardData[4];

  bool _firstRun = true;
  RobotRunner* _robotRunner = nullptr;
  RobotControlParameters _robotParams;
  u64 _iterations = 0;
  std::thread _interfaceLcmThread;
  volatile bool _interfaceLcmQuit = false;
  RobotController* _controller = nullptr;
  ControlParameters* _userControlParameters = nullptr;

  int _port;
};

/*!
 * Interface between robot and hardware specialized for IUST robot
 */
class IUSTrobotHardwareBridge : public HardwareBridge {
public:
  /*!
   * Constructor for IUSTrobotHardwareBridge
   * 
   * @param rc Pointer to the RobotController object
   * @param load_parameters_from_file Boolean value indicating whether to load parameters from a file
   */
  IUSTrobotHardwareBridge (RobotController* rc, bool load_parameters_from_file);
  /*!
   * Runs the SPI
   */
  void runCAN();
  /*!
   * Initializes the hardware
   */
  void initHardware();
  /*!
   * Runs the robot
   */
  void run();
  /*!
   * Runs the Microstrain
   */
  void runMicrostrain();
  /*!
   * Logs the Microstrain data
   */
  void logMicrostrain();

private:
  CAN CANable;
  ImuData _vectorNavData;
  lcm::LCM _spiLcm;
  lcm::LCM _microstrainLcm;
  std::thread* _microstrainThread;
  LordImu _microstrainImu;
  microstrain_lcmt _microstrainData;
  bool _microstrainInit = false;
  bool _load_parameters_from_file;
  u64 spi_times=0;
};

/*!
 * Interface between robot and hardware specialized for Mini Cheetah
 */
class MiniCheetahHardwareBridge : public HardwareBridge {
 public:
  MiniCheetahHardwareBridge(RobotController* rc, bool load_parameters_from_file);
  void runSpi();
  void initHardware();
  void run();
  void runMicrostrain();
  void logMicrostrain();
  void abort(const std::string& reason);
  void abort(const char* reason);

 private:
  ImuData _vectorNavData;
  lcm::LCM _spiLcm;
  lcm::LCM _microstrainLcm;
  std::thread _microstrainThread;
  LordImu _microstrainImu;
  microstrain_lcmt _microstrainData;
  bool _microstrainInit = false;
  bool _load_parameters_from_file;
};

/*!
 * Interface between robot and hardware specialized for Cheetah 3
 */
class Cheetah3HardwareBridge : public HardwareBridge {
public:
  Cheetah3HardwareBridge(RobotController* rc);
  void runEcat();
  void initHardware();
  void run();
  void publishEcatLCM();
  // todo imu?

private:
  ImuData _vectorNavData;
  lcm::LCM _ecatLCM;
  ecat_command_t ecatCmdLcm;
  ecat_data_t ecatDataLcm;
  // nothing?
};
#endif // END of #ifdef linux
#endif  // PROJECT_HARDWAREBRIDGE_H
