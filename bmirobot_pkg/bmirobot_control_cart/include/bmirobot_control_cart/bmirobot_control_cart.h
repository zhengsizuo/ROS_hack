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

#ifndef bmirobot_control_cart_bmirobot_control_cart_H
#define bmirobot_control_cart_bmirobot_control_cart_H

// C++ standard
#include <cassert>
#include <iterator>
#include <stdexcept>
#include <string>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

// ROS
#include <ros/node_handle.h>

// URDF
#include <urdf/model.h>

// ROS messages
#include <control_msgs/FollowJointTrajectoryAction.h>
//#include <control_msgs/BmirobotControlCartState.h>
#include <control_msgs/QueryTrajectoryState.h>
#include <trajectory_msgs/JointTrajectory.h>

// actionlib
#include <actionlib/server/action_server.h>

// realtime_tools
#include <realtime_tools/realtime_box.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

// ros_controls
#include <realtime_tools/realtime_server_goal_handle.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/internal/demangle_symbol.h>

// Project
#include <trajectory_interface/trajectory_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <bmirobot_control_cart/joint_trajectory_segment.h>
#include <bmirobot_control_cart/init_joint_trajectory.h>
#include <bmirobot_control_cart/hardware_interface_adapter.h>
#include <kdl/chainfksolver.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>


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

/**
 * \brief Controller for executing joint-space trajectories on a group of joints.
 *
 * \note Non-developer documentation and usage instructions can be found in the package's ROS wiki page.
 *
 * \tparam SegmentImpl Trajectory segment representation to use. The type must comply with the following structure:
 * \code
 * class FooSegment
 * {
 * public:
 *   // Required types
 *   typedef double                 Scalar; // Scalar can be anything convertible to double
 *   typedef Scalar                 Time;
 *   typedef PosVelAccState<Scalar> State;
 *
 *   // Default constructor
 *   FooSegment();
 *
 *   // Constructor from start and end states (boundary conditions)
 *   FooSegment(const Time&  start_time,
 *              const State& start_state,
 *              const Time&  end_time,
 *              const State& end_state);
 *
 *   // Start and end states initializer (the guts of the above constructor)
 *   // May throw std::invalid_argument if parameters are invalid
 *   void init(const Time&  start_time,
 *             const State& start_state,
 *             const Time&  end_time,
 *             const State& end_state);
 *
 *   // Sampler (realtime-safe)
 *   void sample(const Time& time, State& state) const;
 *
 *   // Accesors (realtime-safe)
 *   Time startTime()    const;
 *   Time endTime()      const;
 *   unsigned int size() const;
 * };
 * \endcode
 *
 * \tparam HardwareInterface Controller hardware interface. Currently \p hardware_interface::PositionJointInterface,
 * \p hardware_interface::VelocityJointInterface, and \p hardware_interface::EffortJointInterface are supported 
 * out-of-the-box.
 */
template <class SegmentImpl, class HardwareInterface>
class BmirobotControlCart : public controller_interface::Controller<HardwareInterface>
{
public:

  BmirobotControlCart();

  /** \name Non Real-Time Safe Functions
   *\{*/
  bool init(HardwareInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  /*\}*/

  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Holds the current position. */
  void starting(const ros::Time& time);

  /** \brief Cancels the active action goal, if any. */
  void stopping(const ros::Time& /*time*/);

  void update(const ros::Time& time, const ros::Duration& period);

  void command(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);

  /*\}*/
    // input of the controller
  KDL::Frame pose_desi_, pose_meas_;
  KDL::Twist twist_ff_;

  // state output
  KDL::Twist twist_error_;

private:
  KDL::Frame getPose();
  struct TimeData
  {
    TimeData() : time(0.0), period(0.0), uptime(0.0) {}

    ros::Time     time;   ///< Time of last update cycle
    ros::Duration period; ///< Period of last update cycle
    ros::Time     uptime; ///< Controller uptime. Set to zero at every restart.
  };
    std::vector<std::string>  joint_names_;  
  KDL::Chain             kdl_chain_;
  boost::scoped_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  KDL::JntArray          jnt_pos_;
  KDL::JntArray       jnt_eff_;
  KDL::Jacobian jacobian_;
// reatltime publisher
  boost::scoped_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist> > state_error_publisher_;
  boost::scoped_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> > state_pose_publisher_;
  unsigned int loop_count_;
  std::string               name_;               ///< Controller name.
  tf::TransformListener tf_;
    ros::NodeHandle    controller_nh_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> sub_command_;
  boost::scoped_ptr<tf::MessageFilter<geometry_msgs::PoseStamped> > command_filter_;
// pid controllers
  std::vector<control_toolbox::Pid> pid_controller_;
    ros::NodeHandle node_;
      std::string controller_name_, root_name_;
        ros::Time last_time_;
        std::vector<hardware_interface::JointHandle>  joints_;             ///< Handles to controlled joints.

};

} // namespace

#include <bmirobot_control_cart/bmirobot_control_cart_impl.h>

#endif // header guard
