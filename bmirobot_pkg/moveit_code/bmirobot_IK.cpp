/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Michael Lautman
   Modified by: Zheng Haosi*/

#include <random>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

using namespace std;

ros::Publisher rightarm_joint_pub;

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
  random_device rd;
  default_random_engine generator{rd()};
  //default_random_engine generator(time(NULL));
  // 0——0.8均匀分布的随机数生成器
  uniform_real_distribution<float> urdx(0.2, 0.5);
  uniform_real_distribution<float> urdy(0.1, 0.4);
  uniform_real_distribution<float> urdz(0.1, 0.3);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  rightarm_joint_pub = n.advertise<std_msgs::Float64MultiArray>( "/bmirobot/right_group_controller/command", 0 );
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

  // Joint Limits
  // ^^^^^^^^^^^^
  // setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
  /* Set one joint in the Panda arm outside its joint limit */
  //joint_values[0] = 1.57;
  //kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

 

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
  float x, y, z;
  T_a.rotate (rotation_vector);
  //T_a.prescale(0.5);  //?
  //T_a.pretranslate(Eigen::Vector3d(0.5, 0.24, 0.1));
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
    x = urdx(generator);
    y = urdy(generator);
    z = urdz(generator);
    cout<<"X:"<<x<<" Y:"<<y<<" Z:"<<z<<endl;
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
      kinematic_state->setToDefaultValues();  // We will set all joints in the state to their default values.
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
