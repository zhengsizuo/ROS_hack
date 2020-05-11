/* Author: Sachin Chitta, Michael Lautman
   Modified by: Zheng Haosi*/

#include <random>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

using namespace std;

ros::Publisher rightarm_joint_pub;
ros::Subscriber pose_sub;
float x, y, z;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
  //pose = pose_msg->pose;
  x = 2*pose_msg->pose.position.x + 0.5;
  y = 3*pose_msg->pose.position.y;
  z = 2*pose_msg->pose.position.z + 0.7;
}
void move_rightarm(float* jointS)
{ 
  //向实体或gazebo机器人发送关节指令
	std_msgs::Float64MultiArray msg;
	for(int i = 0;i< 7;i++)
	{
		msg.data.push_back(jointS[i]);
	}

	rightarm_joint_pub.publish(msg);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_model_and_robot_state_tutorial");
  ros::NodeHandle n("bmirobot");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  rightarm_joint_pub = n.advertise<std_msgs::Float64MultiArray>( "/bmirobot/right_group_controller/command", 0 );
  pose_sub = n.subscribe("/phantom/pose", 1, poseCallback);  //暂定订阅频率为1

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();  // We will set all joints in the state to their default values. 
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("right_arm"); // panda_arm
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  // Get Joint Values
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  } 

  // Forward Kinematics
  // ^^^^^^^^^^^^^^^^^^
  // Now, we can compute forward kinematics for a set of random joint
  // values. Note that we would like to find the pose of the
  // "panda_link8" which is the most distal link in the
  // "panda_arm" group of the robot.

  //kinematic_state->setToRandomPositions(joint_model_group);
  const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("right_link8"); //panda_link8
  
  Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
  Eigen::AngleAxisd rotation_vector(M_PI/4,Eigen::Vector3d(0,0,1));
  cout.precision(3);
  rotation_matrix = rotation_vector.toRotationMatrix(); //旋转向量可以转换为旋转矩阵
  cout<<"rotation_matrix is :"<<endl;
  cout<<rotation_matrix<<endl;
  ///仿射变换
  Eigen::Affine3d T_a = Eigen::Affine3d::Identity();
  // 初始化坐标，关节角为0时的末端位置
  x = 0.304; y = 0.24; z = 0.2;
  /* Print end-effector pose. Remember that this is in the model frame */
  
  ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");  //平移
  ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");  //旋转

  ROS_INFO_STREAM("Translation: \n" << T_a.translation() << "\n");  //平移
  ROS_INFO_STREAM("Rotation: \n" << T_a.rotation() << "\n");  //旋转
  // Inverse Kinematics
  // ^^^^^^^^^^^^^^^^^^
  // We can now solve inverse kinematics (IK) for the Panda robot.
  // To solve IK, we will need the following:
  //
  //  * The desired pose of the end-effector (by default, this is the last link in the "panda_arm" chain):
  //    end_effector_state that we computed in the step above.
  //  * The number of attempts to be made at solving IK: 10
  //  * The timeout for each attempt: 0.1 s
  std::size_t attempts = 10;
  double timeout = 0.1;
  
  //bool found_ik2 = kinematic_state->setFromIK(joint_model_group, end_effector_state, attempts, timeout);
  // Now, we can print out the IK solution (if found):
  while(ros::ok){
    T_a.pretranslate(Eigen::Vector3d(x, y, z));
    bool found_ik = kinematic_state->setFromIK(joint_model_group, T_a, attempts, timeout);
    if (found_ik)
    {
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      float joints[7];
      for (std::size_t i = 0; i < joint_names.size(); ++i)
      {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        joints[i] = joint_values[i];
      }
      move_rightarm(joints); //移动机械臂
    }
    else
    {
      ROS_INFO("Did not find IK solution");
    }
  }

  ros::spinOnce();
  ros::shutdown();
  return 0;
}
