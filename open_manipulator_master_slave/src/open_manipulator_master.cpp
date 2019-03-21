/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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

#include "open_manipulator_master_slave/open_manipulator_master.h"

OpenManipulatorMaster::OpenManipulatorMaster(std::string usb_port, std::string baud_rate)
    :node_handle_(""),
     priv_node_handle_("~"),
     service_call_period_(0.01),
     mode_state_(MASTER_SLAVE_MODE),
     buffer_index_(0)
{
  service_call_period_  = priv_node_handle_.param<double>("service_call_period", 0.010f);
  dxl_id_.push_back(priv_node_handle_.param<int32_t>("joint1_id", 1));
  dxl_id_.push_back(priv_node_handle_.param<int32_t>("joint2_id", 2));
  dxl_id_.push_back(priv_node_handle_.param<int32_t>("joint3_id", 3));
  dxl_id_.push_back(priv_node_handle_.param<int32_t>("joint4_id", 4));
  dxl_id_.push_back(priv_node_handle_.param<int32_t>("gripper_id", 5));

  goal_joint_position_.resize(NUM_OF_JOINT);
  goal_tool_position_ = 0.0;

  initOpenManipulator(usb_port, baud_rate, service_call_period_);
  initServiceClient();

  syncOpenManipulator(false);
}

OpenManipulatorMaster::~OpenManipulatorMaster()
{
  delete actuator_;
  delete tool_;

  if(ros::isStarted()) {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void OpenManipulatorMaster::initServiceClient()
{
  goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
  goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");
}

void OpenManipulatorMaster::initOpenManipulator(STRING usb_port, STRING baud_rate, double service_call_period)
{
  /*****************************************************************************
  ** Initialize Manipulator Parameter
  *****************************************************************************/
  addWorld("world",   // world name
           "joint1"); // child name

  addJoint("joint1",  // my name
           "world",   // parent name
           "joint2",  // child name
           math::vector3(0.012, 0.0, 0.017),                // relative position
           math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
           Z_AXIS,        // axis of rotation
           dxl_id_.at(0), // actuator id
           M_PI,          // max joint limit (3.14 rad)
           -M_PI);        // min joint limit (-3.14 rad)

  addJoint("joint2",  // my name
           "joint1",  // parent name
           "joint3",  // child name
           math::vector3(0.0, 0.0, 0.0595),                 // relative position
           math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
           Y_AXIS,        // axis of rotation
           dxl_id_.at(1), // actuator id
           M_PI_2,        // max joint limit (1.67 rad)
           -2.05);        // min joint limit (-2.05 rad)

  addJoint("joint3",  // my name
           "joint2",  // parent name
           "joint4",  // child name
           math::vector3(0.024, 0.0, 0.128),                // relative position
           math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
           Y_AXIS,        // axis of rotation
           dxl_id_.at(2), // actuator id
           1.53,          // max joint limit (1.53 rad)
           -M_PI_2);      // min joint limit (-1.67 rad)

  addJoint("joint4",  // my name
           "joint3",  // parent name
           "gripper", // child name
           math::vector3(0.124, 0.0, 0.0),                  // relative position
           math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
           Y_AXIS,        // axis of rotation
           dxl_id_.at(3), // actuator id
           2.0,           // max joint limit (2.0 rad)
           -1.8);         // min joint limit (-1.8 rad)

  addTool("gripper",  // my name
          "joint4",   // parent name
          math::vector3(0.126, 0.0, 0.0),                  // relative position
          math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
          dxl_id_.at(4),  // actuator id
          0.010,          // max gripper limit (0.01 m)
          -0.010,         // min gripper limit (-0.01 m)
          -0.015);        // Change unit from `meter` to `radian`

  /*****************************************************************************
  ** Initialize Joint Actuator
  *****************************************************************************/
  actuator_ = new dynamixel::JointDynamixelProfileControl(service_call_period);

  // Set communication arguments
  STRING dxl_comm_arg[2] = {usb_port, baud_rate};
  void *p_dxl_comm_arg = &dxl_comm_arg;

  // Set joint actuator id
  std::vector<uint8_t> jointDxlId;
  jointDxlId.push_back(dxl_id_.at(0));
  jointDxlId.push_back(dxl_id_.at(1));
  jointDxlId.push_back(dxl_id_.at(2));
  jointDxlId.push_back(dxl_id_.at(3));
  addJointActuator(JOINT_DYNAMIXEL, actuator_, jointDxlId, p_dxl_comm_arg);

  // Set joint actuator control mode
  STRING joint_dxl_mode_arg = "position_mode";
  void *p_joint_dxl_mode_arg = &joint_dxl_mode_arg;
  setJointActuatorMode(JOINT_DYNAMIXEL, jointDxlId, p_joint_dxl_mode_arg);

  /*****************************************************************************
  ** Initialize Tool Actuator
  *****************************************************************************/
  tool_ = new dynamixel::GripperDynamixel();

  uint8_t gripperDxlId = dxl_id_.at(4);
  addToolActuator(TOOL_DYNAMIXEL, tool_, gripperDxlId, p_dxl_comm_arg);

  // Set gripper actuator control mode
  STRING gripper_dxl_mode_arg = "current_based_position_mode";
  void *p_gripper_dxl_mode_arg = &gripper_dxl_mode_arg;
  setToolActuatorMode(TOOL_DYNAMIXEL, p_gripper_dxl_mode_arg);

  // Disable All Actuators
  disableAllActuator();
}

void OpenManipulatorMaster::syncOpenManipulator(bool recorded_state)
{
  log::println("Synchronizing Open Manipulators.", "GREEN");
  double sync_path_time = 1.0;

  receiveAllJointActuatorValue();
  receiveAllToolActuatorValue();

  if(recorded_state) // move to first pose of recorded buffer
  {
    setJointSpacePath(sync_path_time, record_buffer_.at(buffer_index_).joint_angle);
    setToolPath(record_buffer_.at(buffer_index_).tool_position);
  }
  else // move to present master pose
  {
    setJointSpacePath(sync_path_time);
    setToolPath();
  }

  ros::WallDuration sleep_time(sync_path_time);
  sleep_time.sleep();

  return;
}

bool OpenManipulatorMaster::setJointSpacePath(double path_time, std::vector<double> set_goal_joint_position)
{
  auto joint_name = getManipulator()->getAllActiveJointComponentName();
  std::vector<double> joint_value;
  if(set_goal_joint_position.size())  joint_value = set_goal_joint_position;
  else
  {
    joint_value.push_back(getJointValue(joint_name.at(0)).position);
    joint_value.push_back(getJointValue(joint_name.at(1)).position);
    joint_value.push_back(getJointValue(joint_name.at(2)).position);
    joint_value.push_back(getJointValue(joint_name.at(3)).position);
  }

  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;

  for(int i = 0; i < NUM_OF_JOINT; i ++)
  {
    if(getManipulator()->checkJointLimit(joint_name.at(i), joint_value.at(i)))
      srv.request.joint_position.position.push_back(joint_value.at(i));
    else
      srv.request.joint_position.position.push_back(goal_joint_position_.at(i));
  }

  goal_joint_position_ = srv.request.joint_position.position;
  srv.request.path_time = path_time;

  if(goal_joint_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorMaster::setToolPath(double set_goal_tool_position)
{
  double tool_value;
  if(set_goal_tool_position < -0.1)
    tool_value = getAllToolValue().at(0).position;
  else
    tool_value = set_goal_tool_position;

  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name.push_back("gripper");

  if(getManipulator()->checkJointLimit("gripper", tool_value))
    srv.request.joint_position.position.push_back(tool_value);
  else
    srv.request.joint_position.position.push_back(goal_tool_position_);

  goal_tool_position_ = srv.request.joint_position.position.at(0);

  if(goal_tool_control_client_.call(srv))
  {
    return srv.response.is_planned;
  }

  return false;
}


void OpenManipulatorMaster::publishCallback(const ros::TimerEvent&)
{
  static int count = 0;
  if(!(count % 5))
  {
    printText();
    if(kbhit())
      setModeState(std::getchar());
  }
  count ++;

  receiveAllJointActuatorValue();
  receiveAllToolActuatorValue();

  if(mode_state_ == MASTER_SLAVE_MODE)
  {
    setJointSpacePath(service_call_period_);
    setToolPath();
  }
  else if(mode_state_ == START_RECORDING_TRAJECTORY_MODE)
  {
    setJointSpacePath(service_call_period_);
    setToolPath();

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
      setJointSpacePath(service_call_period_, record_buffer_.at(buffer_index_).joint_angle);
      setToolPath(record_buffer_.at(buffer_index_).tool_position);
      buffer_index_ ++;
    }
  }
}
void OpenManipulatorMaster::setModeState(char ch)
{
  if(ch == '1')
  {
    syncOpenManipulator(false);
    mode_state_ = MASTER_SLAVE_MODE;
  }
  else if(ch == '2')
  {
    syncOpenManipulator(false);
    record_buffer_.clear();
    mode_state_ = START_RECORDING_TRAJECTORY_MODE;
  }
  else if(ch == '3')
  {
    mode_state_ = STOP_RECORDING_TRAJECTORY_MODE;
  }
  else if(ch == '4')
  {
    buffer_index_ = 0;
    if(record_buffer_.size()) syncOpenManipulator(true);
    mode_state_ = PLAY_RECORDED_TRAJECTORY_MODE;
  }
}

void OpenManipulatorMaster::printText()
{
  system("clear");

  printf("\n");
  printf("-----------------------------\n");
  printf("Control Your OpenManipulator!\n");

  printf("-----------------------------\n");
  printf("Present Control Mode\n");
  if(mode_state_ == MASTER_SLAVE_MODE)
  {
    printf("Master - Slave Mode\n");
  }
  else if(mode_state_ == START_RECORDING_TRAJECTORY_MODE)
  {
    printf("Start Recording Trajectory\n");
    printf("Buffer Size : %d\n", (int)(record_buffer_.size()));
  }
  else if(mode_state_ == STOP_RECORDING_TRAJECTORY_MODE)
  {
    printf("Stop Recording Trajectory\n");
    printf("Buffer Size : %d\n", (int)(record_buffer_.size()));
  }
  else if(mode_state_ == PLAY_RECORDED_TRAJECTORY_MODE)
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
  printf("Present Tool Position: %.3lf\n", goal_tool_position_);
  printf("-----------------------------\n");
}

bool OpenManipulatorMaster::kbhit()
{
  termios term;
  tcgetattr(0, &term);

  termios term2 = term;
  term2.c_lflag &= ~ICANON;
  tcsetattr(0, TCSANOW, &term2);

  int byteswaiting;
  ioctl(0, FIONREAD, &byteswaiting);

  tcsetattr(0, TCSANOW, &term);

  return byteswaiting > 0;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "open_manipulator_master_slave");
  ros::NodeHandle node_handle("");

  std::string usb_port = "/dev/ttyUSB0";
  std::string baud_rate = "1000000";

  if (argc < 3)
  {
    log::error("Please set '-port_name' and  '-baud_rate' arguments for connected Dynamixels");
    return 0;
  }
  else
  {
    usb_port = argv[1];
    baud_rate = argv[2];
  }

  OpenManipulatorMaster open_manipulator_master(usb_port, baud_rate);

  ros::Timer publish_timer = node_handle.createTimer(ros::Duration(open_manipulator_master.getServiceCallPeriod()), &OpenManipulatorMaster::publishCallback, &open_manipulator_master);

  while (ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}
