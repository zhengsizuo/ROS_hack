#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <bmirobot_msg/Robot_ctr.h>
#include <bmirobot_msg/Robot_fdctr.h>
#include <bmirobot_msg/Robot_fdstatus.h>
#include <bmirobot_msg/Robot_mpu.h>
#include <bmirobot_msg/Robot_jointfd.h>
#include "ros/time.h"
#include "ros/duration.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <serial/serial.h>

using namespace std;

#define JOINT_NUM 7
#define HAND_JOINT_NUM 2

#define USE_ODOMETER    0
#define Motor_mode      0x00   //motor mode: (0x00 position) (0x04 speed) (0x08 torque) 0x0c forwardcontorl
#define Motor_status    0x02   //motor status (0x00 normal) (0x02 free) (0x03  hold)

#define pst_mode      0x00
#define spd_mode      0x04
#define tq_mode       0x08
#define fw_mode       0x0c

#define Mnormal    0x00
#define Mfree      0x02
#define Mhold      0x03

#define M1_ctr    	Mnormal	+pst_mode
#define M2_ctr      Mnormal	+pst_mode
#define M3_ctr      Mnormal	+pst_mode
#define M4_ctr      Mnormal	+pst_mode
#define M5_ctr      Mnormal	+pst_mode
#define M6_ctr      Mnormal	+pst_mode
#define M7_ctr      Mnormal	+pst_mode
#define M8_ctr      Mnormal	+pst_mode

using std::string;

class BMIRobot : public hardware_interface::RobotHW
{
public:
  BMIRobot();
  ~BMIRobot();
  void read();
  void write();
  void right_read();
  void right_write();
  void left_read();
  void left_write();
  void initrobot();
  int leftstep1();
  int leftstep2();
  int rightstep1();
  int rightstep2();
  int rightstep3();
  //int writePA();
  ros::Time get_time();
  ros::Duration get_period();
private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double left_cmd[JOINT_NUM+HAND_JOINT_NUM];
  double left_pos[JOINT_NUM+HAND_JOINT_NUM];
  double left_vel[JOINT_NUM+HAND_JOINT_NUM];
  double left_eff[JOINT_NUM+HAND_JOINT_NUM];
  
  double right_cmd[JOINT_NUM+HAND_JOINT_NUM];
  double right_pos[JOINT_NUM+HAND_JOINT_NUM];
  double right_vel[JOINT_NUM+HAND_JOINT_NUM];
  double right_eff[JOINT_NUM+HAND_JOINT_NUM];
  string portNum;
  ros::Time last_time;
  ros::Duration period;
  serial::Serial my_serial;
};


extern void	init_left_robot();
extern void	init_right_robot();
// extern int leftstep1();
// extern int leftstep2();
// extern int rightstep1();
// extern int rightstep2();
// extern int rightstep3();
extern float ljoint_limition[9][2];
extern float rjoint_limition[9][2];
extern void  lFdctrmsgCB(const bmirobot_msg::Robot_fdctr::ConstPtr& msg);
extern void  rFdctrmsgCB(const bmirobot_msg::Robot_fdctr::ConstPtr& msg);
extern bmirobot_msg::Robot_ctr lMTctrmsg;
extern bmirobot_msg::Robot_jointfd lJointfdmsg;
extern ros::Publisher lJoint_fdpub;
extern ros::Publisher lMT_ctrpub;

extern bmirobot_msg::Robot_ctr rMTctrmsg;
extern bmirobot_msg::Robot_jointfd rJointfdmsg;
extern ros::Publisher rJoint_fdpub;
extern ros::Publisher rMT_ctrpub;

