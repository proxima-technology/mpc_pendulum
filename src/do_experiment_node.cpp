#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
//#include "read_write_node.hpp"

#include <iostream>
#include <fstream>
#include <ctime>
#include <casadi/casadi.hpp>
#include <iostream>
#include <cstdlib>

using namespace casadi;

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_CURRENT 102
#define ADDR_PRESENT_CURRENT 126 
#define ADDR_PRESENT_VELOCITY 128
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

#define MOTOR_ID 1

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint8_t dxl_error = 0;
int dxl_comm_result = COMM_TX_FAIL;


void setupDynamixel(uint8_t dxl_id)
{
  // Use Position Control Mode
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_OPERATING_MODE,
    0,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("do_experiment_node"), "Failed to set Current Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("do_experiment_node"), "Succeeded to set Current Control Mode.");
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("do_experiment_node"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("do_experiment_node"), "Succeeded to enable torque.");
  }
}

void read_state(double *position_rad, double *velocity_radpersec)
{
  int present_position_rawdata; // Position [deg] = Value * 360 [deg] / 4096
  int present_velocity_rawdata; // Velocity [rpm] = Value * 0.229 [rpm]

  dxl_comm_result = packetHandler->read4ByteTxRx(
      portHandler,
      MOTOR_ID,
      ADDR_PRESENT_POSITION,
      reinterpret_cast<uint32_t *>(&present_position_rawdata),
      &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    RCLCPP_ERROR(rclcpp::get_logger("do_experiment_node"), "Failed to read present position!");    
  }

  dxl_comm_result = packetHandler->read4ByteTxRx(
      portHandler,
      MOTOR_ID,
      ADDR_PRESENT_VELOCITY,
      reinterpret_cast<uint32_t *>(&present_velocity_rawdata),
      &dxl_error); 
  if (dxl_comm_result != COMM_SUCCESS)
  {
    RCLCPP_ERROR(rclcpp::get_logger("do_experiment_node"), "Failed to read present velocity!");    
  }

  *position_rad = present_position_rawdata * 2.0 * M_PI / 4096;
  *position_rad += +9.*M_PI/180.; // offset
  *velocity_radpersec = present_velocity_rawdata * 0.229 * (M_PI / 30.);
}

void write_control(double current_mA)
{
  double scalingFactor_XM430_W350 = 2.69; // taken from https://www.besttechnology.co.jp/modules/knowledge/?Dynamixel%20XM430-W350#y2cc93fd 
  uint16_t goal_current_rawdata = (uint16_t)(current_mA/scalingFactor_XM430_W350); // Current [mA] = Value * ScalingFactor [mA]

  dxl_comm_result = packetHandler->write2ByteTxRx(
      portHandler,
      MOTOR_ID,
      ADDR_GOAL_CURRENT,
      goal_current_rawdata,
      &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    RCLCPP_ERROR(rclcpp::get_logger("do_experiment_node"), "Failed to write goal current!");    
  }
}

int main(int argc, char * argv[])
{
  double measured_position_rad;
  double measured_velocity_radpersec;
  double smoothed_position_rad;
  double smoothed_velocity_radpersec;
  
  double command_current_mA;
  double current_max_mA = 125.0;

  // ros2 initialize 
  rclcpp::init(argc, argv);
  rclcpp::Clock ros_clock(RCL_ROS_TIME);


  // [dynamixel] initialize
  RCLCPP_INFO(rclcpp::get_logger("do_experiment_node"), "dynamixel initialize start.");

  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // [dynamixel] Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("do_experiment_node"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("do_experiment_node"), "Succeeded to open the port.");
  }

  // [dynamixel] Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("do_experiment_node"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("do_experiment_node"), "Succeeded to set the baudrate.");
  }

  setupDynamixel(MOTOR_ID);  
  read_state(&measured_position_rad, &measured_velocity_radpersec);
  smoothed_position_rad = measured_position_rad;
  smoothed_velocity_radpersec = measured_velocity_radpersec;
  RCLCPP_INFO(rclcpp::get_logger("do_experiment_node"), "dynamixel initialize end.");



  // [MPC] initialize
  RCLCPP_INFO(rclcpp::get_logger("do_experiment_node"), "mpc initialize start.");
  Dict opts;
  opts["print_time"] = false;
  opts["ipopt"] = Dict{{"print_level", 0}};
  std::string lib_nlpsol_swingup_path = std::getenv("COLCON_PREFIX_PATH");
  lib_nlpsol_swingup_path += "/../src/mpc_pendulum/lib/nlpsol_swingup.so";
  Function solver_warmingup = nlpsol("solver", "ipopt", lib_nlpsol_swingup_path, opts);
  opts["ipopt"] = Dict{{"print_level", 0},{"max_iter",3}};
  Function solver_online = nlpsol("solver", "ipopt", lib_nlpsol_swingup_path, opts);
  
  int nx = 2;
  int nh = 50; // horizon
  std::vector<double> x0(nx+(nx+1)*nh, 0);
  std::vector<double> xmin(nx+(nx+1)*nh, -std::numeric_limits<double>::infinity());
  std::vector<double> xmax(nx+(nx+1)*nh, +std::numeric_limits<double>::infinity());
  xmin.at(0) = measured_position_rad; // current state
  xmax.at(0) = measured_position_rad; // current state
  xmin.at(1) = measured_velocity_radpersec; // current state
  xmax.at(1) = measured_velocity_radpersec; // current state
  for(int i=0;i<nh;i++)
  {
    xmin.at(nx*(nh+1)+i) = -current_max_mA; // control limit
    xmax.at(nx*(nh+1)+i) = current_max_mA;  // control limit
  }
  std::vector<double> gmin(nx*nh, 0), gmax(nx*nh, 0);
  std::map<std::string, DM> arg = {{"lbx", xmin}, {"ubx", xmax}, {"lbg", gmin}, {"ubg", gmax}, {"x0",  0}};
  double optimization_start_time = ros_clock.now().nanoseconds()/1.0e9;
  auto res = solver_warmingup(arg); // warmup
  auto res_x = res.at("x");
  auto res_lam_x0 = res.at("lam_x");
  auto res_lam_g0 = res.at("lam_g");
  double previous_optimization_time = ros_clock.now().nanoseconds()/1.0e9 - optimization_start_time; 
  RCLCPP_INFO(rclcpp::get_logger("do_experiment_node"), "mpc warmup optimization time %f [sec]",previous_optimization_time);
  RCLCPP_INFO(rclcpp::get_logger("do_experiment_node"), "mpc initialize end.");

  // [MPC] initialize
  rclcpp::WallRate loop(100); // Hz
  // rclcpp::WallRate loop(25); // Hz
  double worst_case_execution_time = 0.;
  double previous_current_mA = 0.;
  while (rclcpp::ok())
  {
    double loop_start_time = ros_clock.now().nanoseconds()/1.0e9;
    
    read_state(&measured_position_rad, &measured_velocity_radpersec);

    double target_position_rad = 0.;
    
    double diff_rad = measured_position_rad-target_position_rad;
    while(1)
    {
      if(diff_rad>M_PI) diff_rad -= 2*M_PI;
      if(diff_rad<-M_PI) diff_rad += 2*M_PI;
      if(std::abs(diff_rad)<=M_PI) break;
    }

    
    command_current_mA = 0.;

    {
      RCLCPP_INFO(rclcpp::get_logger("do_experiment_node"), 
       "position %f [deg], velocity %f [rad/sec], (previous) current %f [mA], (previous) optimization time %f [sec]", 
       measured_position_rad*180/M_PI, measured_velocity_radpersec, previous_current_mA, previous_optimization_time);

      optimization_start_time = ros_clock.now().nanoseconds()/1.0e9;


      /*
      xmin.at(0) = measured_position_rad; // current state
      xmax.at(0) = measured_position_rad; // current state
      xmin.at(1) = measured_velocity_radpersec; // current state
      xmax.at(1) = measured_velocity_radpersec; // current state
      */

      double decay_rate = 0.3;
      smoothed_position_rad *= decay_rate;
      smoothed_position_rad += ( 1 - decay_rate) * measured_position_rad;
      smoothed_velocity_radpersec *= decay_rate;
      smoothed_velocity_radpersec += ( 1 - decay_rate) * measured_velocity_radpersec;
      xmin.at(0) = smoothed_position_rad; // current state
      xmax.at(0) = smoothed_position_rad; // current state
      xmin.at(1) = smoothed_velocity_radpersec; // current state
      xmax.at(1) = smoothed_velocity_radpersec; // current state

      
      std::map<std::string, DM> temp_arg = {{"lbx", xmin}, {"ubx", xmax}, {"lbg", gmin}, {"ubg", gmax}, {"x0",  res_x}
                                            , {"lam_x0",  res_lam_x0}, {"lam_g0",  res_lam_g0}
                                           };
      res = solver_online(temp_arg);
      res_x = res.at("x");
      res_lam_x0 = res.at("lam_x");
      res_lam_g0 = res.at("lam_g");
      previous_optimization_time = ros_clock.now().nanoseconds()/1.0e9 - optimization_start_time;
      command_current_mA = (double) res_x(nx*(nh+1));
    }

    command_current_mA = std::max(-current_max_mA, std::min(current_max_mA, command_current_mA));
    previous_current_mA = command_current_mA;
    
    write_control(command_current_mA);

    double loop_end_time = ros_clock.now().nanoseconds()/1.0e9 - loop_start_time;
    if (loop_end_time>worst_case_execution_time) worst_case_execution_time = loop_end_time;
    loop.sleep();
  }
  RCLCPP_INFO(rclcpp::get_logger("do_experiment_node"), "worst case execution time: %f [sec]", worst_case_execution_time);
  rclcpp::shutdown();
  

  // [dynamixel] Disable Torque of DYNAMIXEL
  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );

  return 0;
}
