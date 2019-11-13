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

#include "open_manipulator_x_master_slave/open_manipulator_x_master_slave.hpp"

using namespace std::chrono_literals;


OpenManipulatorXMasterSlave::OpenManipulatorXMasterSlave(std::string usb_port, std::string baud_rate)
: Node("open_manipulator_x_master_slave")
{
  /************************************************************
  ** Initialise ROS parameters
  ************************************************************/
  init_parameters();

  /************************************************************
  ** Initialise variables
  ************************************************************/
  goal_joint_position_.resize(NUM_OF_JOINT);
  goal_tool_position_ = 0.0;

  open_manipulator_x_.init_open_manipulator_x(true, usb_port, baud_rate, service_call_period_, dxl_id_);
  open_manipulator_x_.disableAllActuator();

  mode_state_ = MASTER_SLAVE_MODE;
  buffer_index_ = 0;
  
  this->disable_waiting_for_enter();

  /************************************************************
  ** Initialise ROS clients
  ************************************************************/
  goal_joint_space_path_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>(
    "open_manipulator_x/goal_joint_space_path");
  goal_tool_control_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>(
    "open_manipulator_x/goal_tool_control");

  /************************************************************
  ** Initialise ROS timers
  ************************************************************/
  update_timer_ = this->create_wall_timer(
    10ms, std::bind(&OpenManipulatorXMasterSlave::update_callback, this));  

  RCLCPP_INFO(this->get_logger(), "OpenManipulator-X master slave node has been initialised.");
}

OpenManipulatorXMasterSlave::~OpenManipulatorXMasterSlave()
{
  this->restore_terminal_settings();
  RCLCPP_INFO(this->get_logger(), "OpenManipulator-X master slave node has been terminated.");
}

/********************************************************************************
** Init Functions
********************************************************************************/
void OpenManipulatorXMasterSlave::init_parameters()
{
  // Declare parameters that may be set on this node
  this->declare_parameter("service_call_period");
  this->declare_parameter("joint1_id");
  this->declare_parameter("joint2_id");
  this->declare_parameter("joint3_id");
  this->declare_parameter("joint4_id");
  this->declare_parameter("gripper_id");

  // Get parameter from yaml
  int dxl_id_1, dxl_id_2, dxl_id_3, dxl_id_4, dxl_id_5;
  this->get_parameter_or<double>("service_call_period", service_call_period_, 0.010);
  this->get_parameter_or<int>("joint1_id", dxl_id_1, 1);
  this->get_parameter_or<int>("joint2_id", dxl_id_2, 2);
  this->get_parameter_or<int>("joint3_id", dxl_id_3, 3);
  this->get_parameter_or<int>("joint4_id", dxl_id_4, 4);
  this->get_parameter_or<int>("gripper_id", dxl_id_5, 5);
  dxl_id_.push_back(dxl_id_1);
  dxl_id_.push_back(dxl_id_2);
  dxl_id_.push_back(dxl_id_3);
  dxl_id_.push_back(dxl_id_4);
  dxl_id_.push_back(dxl_id_5);
}

/*****************************************************************************
** Callback functions for ROS clients
*****************************************************************************/
void OpenManipulatorXMasterSlave::sync_open_manipulator_x(bool recorded_state)
{
  RCLCPP_INFO(this->get_logger(), "Synchronizing OpenManipulator-X");
  double sync_path_time = 1.0;

  open_manipulator_x_.receiveAllJointActuatorValue();
  open_manipulator_x_.receiveAllToolActuatorValue();

  if (recorded_state) // move to first pose of recorded buffer
  {
    set_joint_space_path(sync_path_time, record_buffer_.at(buffer_index_).joint_angle);
    set_tool_control(record_buffer_.at(buffer_index_).tool_position);
  }
  else // move to present master pose
  {
    set_joint_space_path(sync_path_time);
    set_tool_control();
  }

  return;
}

void OpenManipulatorXMasterSlave::set_goal()
{
  if(mode_state_ == MASTER_SLAVE_MODE)
  {
    set_joint_space_path(service_call_period_);
    set_tool_control();
  }
  else if(mode_state_ == START_RECORDING_TRAJECTORY_MODE)
  {
    set_joint_space_path(service_call_period_);
    set_tool_control();

    WaypointBuffer temp;
    temp.joint_angle = goal_joint_position_;
    temp.tool_position = goal_tool_position_;
    record_buffer_.push_back(temp);
  }
  else if(mode_state_ == STOP_RECORDING_TRAJECTORY_MODE)
  {}
  else if(mode_state_ == PLAY_RECORDED_TRAJECTORY_MODE)
  {
    if(record_buffer_.size() > buffer_index_)
    {
      set_joint_space_path(service_call_period_, record_buffer_.at(buffer_index_).joint_angle);
      set_tool_control(record_buffer_.at(buffer_index_).tool_position);
      buffer_index_ ++;
    }
  }
}

bool OpenManipulatorXMasterSlave::set_joint_space_path(double path_time, std::vector<double> joint_angle)
{
  auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();

  std::vector<std::string> joint_name = open_manipulator_x_.getManipulator()->getAllActiveJointComponentName();
  request->joint_position.joint_name = joint_name;

  std::vector<double> joint_value;
  if (joint_angle.size())  
    joint_value = joint_angle;
  else
  {
    joint_value.push_back(open_manipulator_x_.getJointValue(joint_name.at(0)).position);
    joint_value.push_back(open_manipulator_x_.getJointValue(joint_name.at(1)).position);
    joint_value.push_back(open_manipulator_x_.getJointValue(joint_name.at(2)).position);
    joint_value.push_back(open_manipulator_x_.getJointValue(joint_name.at(3)).position);
  }

  for (int i = 0; i < NUM_OF_JOINT; i ++)
  {
    if(open_manipulator_x_.getManipulator()->checkJointLimit(joint_name.at(i), joint_value.at(i)))
      request->joint_position.position.push_back(joint_value.at(i));
    else
      request->joint_position.position.push_back(goal_joint_position_.at(i));
  }

  goal_joint_position_ = request->joint_position.position;

  request->path_time = path_time;

  using ServiceResponseFuture = rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) 
  {
    auto result = future.get();
    return result->is_planned;
  };
  auto future_result = goal_joint_space_path_client_->async_send_request(request, response_received_callback);

  return false;
}

bool OpenManipulatorXMasterSlave::set_tool_control(double set_goal_tool_position)
{
  double tool_value;
  if(set_goal_tool_position < -0.1)
    tool_value = open_manipulator_x_.getAllToolValue().at(0).position;
  else
    tool_value = set_goal_tool_position;

  auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
  request->joint_position.joint_name.push_back("gripper");

  if (open_manipulator_x_.getManipulator()->checkJointLimit("gripper", tool_value))
    request->joint_position.position.push_back(tool_value);
  else
    request->joint_position.position.push_back(goal_tool_position_);

  using ServiceResponseFuture = rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) 
  {
    auto result = future.get();
    return result->is_planned;
  };
  auto future_result = goal_tool_control_client_->async_send_request(request, response_received_callback);


  return false;
}

void OpenManipulatorXMasterSlave::update_callback()
{
  open_manipulator_x_.receiveAllJointActuatorValue();
  open_manipulator_x_.receiveAllToolActuatorValue();

  this->print_text();
  
  char ch = std::getchar();
  if (ch=='1' || ch=='2' || ch=='3' || ch=='4')
    this->set_mode_state(ch);

  this->set_goal();
}

void OpenManipulatorXMasterSlave::set_mode_state(char ch)
{
  if (ch == '1')
  {
    sync_open_manipulator_x(false);
    mode_state_ = MASTER_SLAVE_MODE;
  }
  else if (ch == '2')
  {
    sync_open_manipulator_x(false);
    record_buffer_.clear();
    mode_state_ = START_RECORDING_TRAJECTORY_MODE;
  }
  else if (ch == '3')
  {
    mode_state_ = STOP_RECORDING_TRAJECTORY_MODE;
  }
  else if (ch == '4')
  {
    buffer_index_ = 0;
    if (record_buffer_.size()) sync_open_manipulator_x(true);
    mode_state_ = PLAY_RECORDED_TRAJECTORY_MODE;
  }
}

/********************************************************************************
** Other Functions
********************************************************************************/
void OpenManipulatorXMasterSlave::print_text()
{
  system("clear");
  printf("\n");
  printf("-----------------------------\n");
  printf("Control Your OpenManipulator!\n");
  printf("-----------------------------\n");
  printf("Present Control Mode\n");

  if (mode_state_ == MASTER_SLAVE_MODE)
  {
    printf("Master - Slave Mode\n");
  }
  else if (mode_state_ == START_RECORDING_TRAJECTORY_MODE)
  {
    printf("Start Recording Trajectory\n");
    printf("Buffer Size : %d\n", (int)(record_buffer_.size()));
  }
  else if (mode_state_ == STOP_RECORDING_TRAJECTORY_MODE)
  {
    printf("Stop Recording Trajectory\n");
    printf("Buffer Size : %d\n", (int)(record_buffer_.size()));
  }
  else if (mode_state_ == PLAY_RECORDED_TRAJECTORY_MODE)
  {
    printf("Play Recorded Trajectory Mode\n");
    printf("Buffer Size : %d\n", (int)(record_buffer_.size()));
    printf("Buffer index : %d\n", buffer_index_);
  }

  printf("-----------------------------\n");
  printf("1 : Master - Slave Mode\n");
  printf("2 : Start Recording Trajectory\n");
  printf("3 : Stop Recording Trajectory\n");
  printf("4 : Play Recorded Trajectory\n");
  printf("-----------------------------\n");
  printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
    goal_joint_position_.at(0),
    goal_joint_position_.at(1),
    goal_joint_position_.at(2),
    goal_joint_position_.at(3));
  printf("Present Tool Position: %.3lf\n", 
    goal_tool_position_);
  printf("-----------------------------\n");
}

void OpenManipulatorXMasterSlave::restore_terminal_settings()
{
  tcsetattr(0, TCSANOW, &oldt_);  /* Apply saved settings */
}

void OpenManipulatorXMasterSlave::disable_waiting_for_enter()
{
  struct termios newt;

  tcgetattr(0, &oldt_);             /* Save terminal settings */
  newt = oldt_;                     /* Init new settings */
  newt.c_lflag &= ~(ICANON | ECHO); /* Change settings */
  newt.c_cc[VMIN] = 0;
  newt.c_cc[VTIME] = 0;
  tcsetattr(0, TCSANOW, &newt);     /* Apply settings */
}

/*****************************************************************************
** Main
*****************************************************************************/
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::string usb_port = "/dev/ttyUSB1";
  std::string baud_rate = "1000000";

  if (argc == 3)
  {
    usb_port = argv[1];
    baud_rate = argv[2];
    printf("port_name and baud_rate are set to %s, %s \n", usb_port.c_str(), baud_rate.c_str());
  }
  else
    printf("port_name and baud_rate are set to %s, %s \n", usb_port.c_str(), baud_rate.c_str());

  rclcpp::spin(std::make_shared<OpenManipulatorXMasterSlave>(usb_port, baud_rate));
  rclcpp::shutdown();

  return 0;
}
