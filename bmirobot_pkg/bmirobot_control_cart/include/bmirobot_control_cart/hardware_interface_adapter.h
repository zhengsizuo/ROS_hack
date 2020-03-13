///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
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

/// \author Adolfo Rodriguez Tsouroukdissian

#ifndef bmirobot_control_cart_HARDWARE_INTERFACE_ADAPTER_H
#define bmirobot_control_cart_HARDWARE_INTERFACE_ADAPTER_H

#include <cassert>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <ros/node_handle.h>
#include <ros/time.h>

#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>

/**
 * \brief Helper class to simplify integrating the BmirobotControlCart with different hardware interfaces.
 *
 * The BmirobotControlCart outputs position, velocity and acceleration command triplets, while the more common hardware
 * interfaces accept position, velocity or effort commands.
 *
 * Use one of the available template specializations of this class (or create your own) to adapt the
 * BmirobotControlCart to a specidfic hardware interface.
 */
template <class HardwareInterface, class State>
class HardwareInterfaceAdapter
{
public:
  bool init(std::vector<typename HardwareInterface::ResourceHandleType>& /*joint_handles*/, ros::NodeHandle& /*controller_nh*/)
  {
    return false;
  }

  void starting(const ros::Time& /*time*/) {}
  void stopping(const ros::Time& /*time*/) {}

  void updateCommand(const ros::Time&     /*time*/,
                     const ros::Duration& /*period*/,
                     const State&         /*desired_state*/,
                     const State&         /*state_error*/) {}
};

/**
 * \brief Adapter for a position-controlled hardware interface. Forwards desired positions as commands.
 *
 * The following is an example configuration of a controller that uses this adapter.
 * \code
 * head_controller:
 *   type: "position_controllers/BmirobotControlCart"
 *   joints:
 *     - head_1_joint
 *     - head_2_joint
 *   
 *   constraints:
 *     goal_time: 0.6
 *     stopped_velocity_tolerance: 0.02
 *     head_1_joint: {trajectory: 0.05, goal: 0.02}
 *     head_2_joint: {trajectory: 0.05, goal: 0.02}
 *   stop_trajectory_duration: 0.5
 *   state_publish_rate:  25
 * \endcode
 */
template <class State>
class HardwareInterfaceAdapter<hardware_interface::PositionJointInterface, State>
{
public:
  HardwareInterfaceAdapter() : joint_handles_ptr_(0) {}

  bool init(std::vector<hardware_interface::JointHandle>& joint_handles, ros::NodeHandle& /*controller_nh*/)
  {
    // Store pointer to joint handles
    joint_handles_ptr_ = &joint_handles;

    return true;
  }

  void starting(const ros::Time& /*time*/)
  {
    if (!joint_handles_ptr_) {return;}

    // Semantic zero for commands
    for (unsigned int i = 0; i < joint_handles_ptr_->size(); ++i)
    {
      (*joint_handles_ptr_)[i].setCommand((*joint_handles_ptr_)[i].getPosition());
    }
  }

  void stopping(const ros::Time& /*time*/) {}

  void updateCommand(const ros::Time&     /*time*/,
                     const ros::Duration& /*period*/,
                     const State&         desired_state,
                     const State&         /*state_error*/)
  {
    // Forward desired position to command
    const unsigned int n_joints = joint_handles_ptr_->size();
    for (unsigned int i = 0; i < n_joints; ++i) {(*joint_handles_ptr_)[i].setCommand(desired_state.position[i]);}
  }

private:
  std::vector<hardware_interface::JointHandle>* joint_handles_ptr_;
};



#endif // header guard
