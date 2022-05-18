/*
 *  Copyright (c) 2022 Ivo Dekker ACRO Diepenbeek KULeuven

 Permission is hereby granted, free of charge, to any person
 obtaining a copy of this software and associated documentation
 files (the "Software"), to deal in the Software without
 restriction, including without limitation the rights to use,
 copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the
 Software is furnished to do so, subject to the following
 conditions:

 The above copyright notice and this permission notice shall be
 included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef FLATHEADERS
#include "simple_message/joint_traj_pt.hpp"
#include "simple_message/shared_types.hpp"
#else
#include "joint_traj_pt.hpp"
#include "shared_types.hpp"
#endif
#include "rclcpp/rclcpp.hpp"

using namespace industrial::joint_data;
using namespace industrial::shared_types;

namespace industrial
{
namespace joint_traj_pt
{

JointTrajPt::JointTrajPt(void)
{
  this->init();
}
JointTrajPt::~JointTrajPt(void)
{

}

void JointTrajPt::init()
{
  this->joint_position_.init();
  this->sequence_ = 0;
  this->velocity_ = 0.0;
  this->duration_ = 0.0;
}

void JointTrajPt::init(shared_int sequence, JointData & position, shared_real velocity, shared_real duration)
{
  this->setJointPosition(position);
  this->setSequence(sequence);
  this->setVelocity(velocity);
  this->setDuration(duration);
}

void JointTrajPt::copyFrom(JointTrajPt &src)
{
  this->setSequence(src.getSequence());
  src.getJointPosition(this->joint_position_);
  this->setVelocity(src.getVelocity());
  this->setDuration(src.getDuration());
}

bool JointTrajPt::operator==(JointTrajPt &rhs)
{
  return this->joint_position_ == rhs.joint_position_ && this->sequence_ == rhs.sequence_
      && this->velocity_ == rhs.velocity_ && this->duration_ == rhs.duration_;

}

bool JointTrajPt::load(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;

  //RCLCPP_INFO(rclcpp::get_logger("joint_traj_pt"), "Executing joint trajectory point load");

  if (buffer->load(this->sequence_))
  {
    if (this->joint_position_.load(buffer))
    {
      if (buffer->load(this->velocity_))
      {
        if (buffer->load(this->duration_))
        {
          //RCLCPP_INFO(rclcpp::get_logger("joint_traj_pt"), "Trajectory point successfully loaded");
          rtn = true;
        }
        else
        {
          rtn = false;
          //RCLCPP_INFO(rclcpp::get_logger("joint_traj_pt"), "Failed to load joint traj pt. duration");
        }
      }
      else
      {
        rtn = false;
        //RCLCPP_ERROR(rclcpp::get_logger("joint_traj_pt"), "Failed to load joint traj pt. velocity");
      }

    }
    else
    {
      rtn = false;
      //RCLCPP_ERROR(rclcpp::get_logger("joint_traj_pt"), "Failed to load joint traj. pt.  position data");
    }
  }
  else
  {
    rtn = false;
    //RCLCPP_ERROR(rclcpp::get_logger("joint_traj_pt"), "Failed to load joint traj. pt. sequence number");
  }

  return rtn;
}

bool JointTrajPt::unload(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;

  //RCLCPP_INFO(rclcpp::get_logger("joint_traj_pt"), "Executing joint traj. pt. unload");
  if (buffer->unload(this->duration_))
  {
    if (buffer->unload(this->velocity_))
    {
      if (this->joint_position_.unload(buffer))
      {
        if (buffer->unload(this->sequence_))
        {
          rtn = true;
          //RCLCPP_INFO(rclcpp::get_logger("joint_traj_pt"), "Joint traj. pt successfully unloaded");
        }
        else
        {
          //RCLCPP_ERROR(rclcpp::get_logger("joint_traj_pt"), "Failed to unload joint traj. pt. sequence number");
          rtn = false;
        }
      }
      else
      {
        //RCLCPP_ERROR(rclcpp::get_logger("joint_traj_pt"), "Failed to unload joint traj. pt.  position data");
        rtn = false;
      }

    }
    else
    {
      //RCLCPP_ERROR(rclcpp::get_logger("joint_traj_pt"), "Failed to unload joint traj. pt. velocity");
      rtn = false;
    }
  }
  else
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_traj_pt"), "Failed to unload joint traj. pt. duration");
    rtn = false;
  }

  return rtn;
}

}
}

