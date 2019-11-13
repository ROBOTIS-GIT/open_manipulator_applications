/*******************************************************************************
* Copyright 2019 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#ifndef OPEN_MANIPULATOR_X_MASTER_SLAVE_HPP
#define OPEN_MANIPULATOR_X_MASTER_SLAVE_HPP

#include <termios.h>
#include <sys/ioctl.h>

#include <rclcpp/rclcpp.hpp>

#include "open_manipulator_msgs/srv/set_joint_position.hpp"
#include "open_manipulator_x_libs/open_manipulator_x.hpp"

#define NUM_OF_JOINT 4

#define MASTER_SLAVE_MODE 0
#define START_RECORDING_TRAJECTORY_MODE 1
#define STOP_RECORDING_TRAJECTORY_MODE 2
#define PLAY_RECORDED_TRAJECTORY_MODE 3


typedef struct _WaypointBuffer
{
  std::vector<double> joint_angle;
  double tool_position;
} WaypointBuffer;

class OpenManipulatorXMasterSlave : public rclcpp::Node
{
 public:
  OpenManipulatorXMasterSlave(std::string usb_port, std::string baud_rate);
  ~OpenManipulatorXMasterSlave();

  void update_callback();  
  
  rclcpp::TimerBase::SharedPtr update_timer_;

 private:
  /*****************************************************************************
  ** ROS Parameters
  *****************************************************************************/
  uint8_t mode_state_;
  double service_call_period_;
  std::vector<uint8_t> dxl_id_;
  std::vector<double> goal_joint_position_;
  double goal_tool_position_;

  /*****************************************************************************
  ** Variables
  *****************************************************************************/
  // Robotis_manipulator related 
  OpenManipulatorX open_manipulator_x_;

  std::vector<WaypointBuffer> record_buffer_;
  int buffer_index_;

  /*****************************************************************************
  ** Init Functions
  *****************************************************************************/
  void init_parameters();

  /*****************************************************************************
  ** ROS Clients
  *****************************************************************************/
  rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_joint_space_path_client_;
  rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_tool_control_client_;

  /*****************************************************************************
  ** Others
  *****************************************************************************/
  void init_open_manipulator_x(STRING usb_port = "/dev/ttyUSB0", STRING baud_rate = "1000000", double service_call_period = 0.010);
  void sync_open_manipulator_x(bool recorded_state);
  double get_service_call_period() {return service_call_period_;}

  void set_goal();
  bool set_joint_space_path(double path_time, std::vector<double> set_goal_joint_position = {});
  bool set_tool_control(double set_goal_tool_position = -1.0);
  void set_mode_state(char ch);

  void print_text();
  struct termios oldt_;
  void restore_terminal_settings();
  void disable_waiting_for_enter();
};
#endif //OPEN_MANIPULATOR_X_MASTER_SLAVE_HPP
