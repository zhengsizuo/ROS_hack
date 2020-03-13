///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
// Copyright (c) 2008, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Adolfo Rodriguez Tsouroukdissian, Stuart Glaser

#ifndef bmirobot_control_cart_bmirobot_control_cart_IMP_H
#define bmirobot_control_cart_bmirobot_control_cart_IMP_H

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <string>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <tf_conversions/tf_kdl.h>


#include <vector>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>

#include <ros/node_handle.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <control_toolbox/pid.h>
#include <kdl/chainfksolver.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <message_filters/subscriber.h>

#include <realtime_tools/realtime_publisher.h>
#include <tf/message_filter.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

namespace bmirobot_control_cart
{

namespace internal
{

std::vector<std::string> getStrings(const ros::NodeHandle& nh, const std::string& param_name)
{
  using namespace XmlRpc;
  XmlRpcValue xml_array;
  if (!nh.getParam(param_name, xml_array))
  {
    ROS_ERROR_STREAM("Could not find '" << param_name << "' parameter (namespace: " << nh.getNamespace() << ").");
    return std::vector<std::string>();
  }
  if (xml_array.getType() != XmlRpcValue::TypeArray)
  {
    ROS_ERROR_STREAM("The '" << param_name << "' parameter is not an array (namespace: " <<
                     nh.getNamespace() << ").");
    return std::vector<std::string>();
  }

  std::vector<std::string> out;
  for (int i = 0; i < xml_array.size(); ++i)
  {
    if (xml_array[i].getType() != XmlRpcValue::TypeString)
    {
      ROS_ERROR_STREAM("The '" << param_name << "' parameter contains a non-string element (namespace: " <<
                       nh.getNamespace() << ").");
      return std::vector<std::string>();
    }
    out.push_back(static_cast<std::string>(xml_array[i]));
  }
  return out;
}
std::string getLeafNamespace(const ros::NodeHandle& nh)
{
  const std::string complete_ns = nh.getNamespace();
  std::size_t id   = complete_ns.find_last_of("/");
  return complete_ns.substr(id + 1);
}


} // namespace

template <class SegmentImpl, class HardwareInterface>
inline void BmirobotControlCart<SegmentImpl, HardwareInterface>::
starting(const ros::Time& time)
{
  // reset pid controllers
  for (unsigned int i=0; i<6; i++)
    pid_controller_[i].reset();

  // initialize desired pose/twist
  twist_ff_ = KDL::Twist::Zero();
  pose_desi_ = getPose();
  last_time_ = ros::Time::now();

  loop_count_ = 0;
}

template <class SegmentImpl, class HardwareInterface>
inline void BmirobotControlCart<SegmentImpl, HardwareInterface>::
stopping(const ros::Time& /*time*/)
{
  
}

template <class SegmentImpl, class HardwareInterface>
KDL::Frame BmirobotControlCart<SegmentImpl, HardwareInterface>::getPose()
{
  // get the joint positions and velocities
  //////chain_.getPositions(jnt_pos_);
for (unsigned int i = 0; i < joints_.size(); ++i)
  {
    jnt_pos_(i) = joints_[i].getPosition();
  }
  // get cartesian pose
  KDL::Frame result;
  jnt_to_pose_solver_->JntToCart(jnt_pos_, result);
  //ROS_ERROR_STREAM("current pose"<< result.p.x()<<"   "<<     result.p.y()<<"   "<<result.p.z()); 
  return result;
}

template <class SegmentImpl, class HardwareInterface>
BmirobotControlCart<SegmentImpl, HardwareInterface>::
BmirobotControlCart()
{}
template <class SegmentImpl, class HardwareInterface>
void BmirobotControlCart<SegmentImpl, HardwareInterface>::
 command(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
  // convert message to transform
  tf::Stamped<tf::Pose> pose_stamped;
  tf::poseStampedMsgToTF(*pose_msg, pose_stamped);

  // convert to reference frame of root link of the controller chain
  tf_.transformPose(root_name_, pose_stamped, pose_stamped);
  tf::poseTFToKDL(pose_stamped, pose_desi_);
  //ROS_ERROR_STREAM("receive command pose_stamped"<<pose_stamped.getOrigin ().getX()<<"   "<< pose_stamped.getOrigin ().getY()
  //  << "  " << pose_stamped.getOrigin ().getZ()<<"pose_desi_" <<
  //   pose_desi_.p.x()); 
}
template <class SegmentImpl, class HardwareInterface>
bool BmirobotControlCart<SegmentImpl, HardwareInterface>::init(HardwareInterface* hw,
                                                                     ros::NodeHandle&   root_nh,
                                                                     ros::NodeHandle&   controller_nh)
{

  using namespace internal;
  node_ = controller_nh;
   KDL::Tree kdl_tree;
   ros::NodeHandle node;
   std::string robot_desc_string;
   controller_nh_=controller_nh;
   node.param("robot_description", robot_desc_string, std::string());
   if (!kdl_parser::treeFromString(robot_desc_string, kdl_tree)){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
   }
   root_name_ = "base_link";
    bool res;
  try{
    res = kdl_tree.getChain("base_link", "link8", kdl_chain_);
  }
  catch(...){
    res = false;
  }
  jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
  jnt_pos_.resize(kdl_chain_.getNrOfJoints());
  jnt_eff_.resize(kdl_chain_.getNrOfJoints());
  jacobian_.resize(kdl_chain_.getNrOfJoints());

  // create pid controller for the translation and for the rotation
  control_toolbox::Pid pid_controller;
  if (!pid_controller.init(ros::NodeHandle(node_,"fb_trans"))) return false;
  for (unsigned int i = 0; i < 3; i++)
    pid_controller_.push_back(pid_controller);
  if (!pid_controller.init(ros::NodeHandle(node_,"fb_rot"))) return false;
  for (unsigned int i = 0; i < 3; i++)
    pid_controller_.push_back(pid_controller);


joints_.resize(kdl_chain_.getNrOfJoints());
joint_names_ = getStrings(controller_nh_, "joints");
name_ = getLeafNamespace(controller_nh_);
for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
  {
    // Joint handle
    try {joints_[i] = hw->getHandle(joint_names_[i]);}
    catch (...)
    {
      ROS_ERROR_STREAM_NAMED(name_, "Could not find joint '" << joint_names_[i] << "' in '" <<
                                    this->getHardwareInterfaceType() << "'.");
      return false;
    }
  }


  // subscribe to pose commands
  sub_command_.subscribe(node_, "command", 10);
  command_filter_.reset(new tf::MessageFilter<geometry_msgs::PoseStamped>(
                          sub_command_, tf_, root_name_, 10, node_));
  command_filter_->registerCallback(boost::bind(&BmirobotControlCart::command, this, _1));

  // realtime publisher for control state
  state_error_publisher_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Twist>(node_, "state/error", 1));
  state_pose_publisher_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>(node_, "state/pose", 1));

  return true;
}

template <class SegmentImpl, class HardwareInterface>
void BmirobotControlCart<SegmentImpl, HardwareInterface>::
update(const ros::Time& time, const ros::Duration& period)
{
  //ROS_ERROR_STREAM("update"); 
  // get time
  ros::Time time2 = ros::Time::now();
  ros::Duration dt = time2 - last_time_;
  last_time_ = time2;

  // get current pose
  pose_meas_ = getPose();

  // pose feedback into twist
  twist_error_ = KDL::diff(pose_meas_, pose_desi_);
//ROS_ERROR_STREAM("pose_desi_"<< pose_desi_.p.x()<<"   "<<     pose_desi_.p.y()<<"   "<<pose_desi_.p.z()); 
  //ROS_ERROR_STREAM("twist_error_"<< twist_error_.vel.x()<<"   "<<     twist_error_.vel.y()<<"   "<<twist_error_.vel.z()); 
  KDL::Wrench wrench_desi;
  for (unsigned int i=0; i<6; i++)
  {
    if(i < 3)
    wrench_desi(i) = pid_controller_[i].computeCommand(twist_error_(i), dt);
  else
    wrench_desi(i) = pid_controller_[i].computeCommand(0, dt);
  }
  //ROS_ERROR_STREAM("wrench_desi"<< dt<<" "<< wrench_desi(0)<<"   "<<    wrench_desi(1)<<"   "<<wrench_desi(2)); 

  // get the chain jacobian
  jac_solver_->JntToJac(jnt_pos_, jacobian_);

  // Converts the wrench into joint efforts with a jacbobian-transpose
  for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++){
    jnt_eff_(i) = 0;
    for (unsigned int j=0; j<6; j++)
      jnt_eff_(i) += (jacobian_(j,i) * wrench_desi(j));
  }
  //ROS_ERROR_STREAM("jnt_eff_"<< jnt_eff_(0)<<"   "<<    jnt_eff_(1)<<"   "<<jnt_eff_(2)); 

  // set effort to joints
  //////chain_.addEfforts(jnt_eff_);
  for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++){
    jnt_eff_(i) *= 0.06;
    //ROS_ERROR_STREAM("joints_[i] "<<jnt_eff_(i)); 
    joints_[i].setCommand(jnt_pos_(i)+jnt_eff_(i));
  }

//hw_iface_adapter_.updateCommand(time_data.uptime, time_data.period,
 //                                 desired_state_, state_error_);

  if (++loop_count_ % 100 == 0){
    if (state_error_publisher_){
      if (state_error_publisher_->trylock()){
        state_error_publisher_->msg_.linear.x = twist_error_.vel(0);
        state_error_publisher_->msg_.linear.y = twist_error_.vel(1);
        state_error_publisher_->msg_.linear.z = twist_error_.vel(2);
        state_error_publisher_->msg_.angular.x = twist_error_.rot(0);
        state_error_publisher_->msg_.angular.y = twist_error_.rot(1);
        state_error_publisher_->msg_.angular.z = twist_error_.rot(2);
        state_error_publisher_->unlockAndPublish();
      }
    }
    if (state_pose_publisher_){
      if (state_pose_publisher_->trylock()){
  tf::Pose tmp;
        tf::poseKDLToTF(pose_meas_, tmp);
  poseStampedTFToMsg(tf::Stamped<tf::Pose>(tmp, ros::Time::now(), root_name_), state_pose_publisher_->msg_);
        state_pose_publisher_->unlockAndPublish();
      }
    }
  }
}



} // namespace

#endif // header guard
