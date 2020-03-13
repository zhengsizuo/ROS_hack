#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "ros/time.h"
#include "ros/duration.h"
#include <serial/serial.h>
#define JOINT_NUM 7
#define HAND_JOINT_NUM 2
using std::string;
class BMIRobot : public hardware_interface::RobotHW
{
public:
  BMIRobot();
  ~BMIRobot();
  void read();
  void read1();
  void write(double* a,bool useF=false);
  int initrobot();
  //int writePA();
  ros::Time get_time();
  ros::Duration get_period();
private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[JOINT_NUM+HAND_JOINT_NUM];
  double pos[JOINT_NUM+HAND_JOINT_NUM];
  double vel[JOINT_NUM+HAND_JOINT_NUM];
  double eff[JOINT_NUM+HAND_JOINT_NUM];
  string portNum;
  ros::Time last_time;
  ros::Duration period;
  serial::Serial my_serial;
};
