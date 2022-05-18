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
#include "simple_message/joint_traj_pt_full.hpp"
#include "simple_message/shared_types.hpp"
#else
#include "joint_traj_pt_full.hpp"
#include "shared_types.hpp"
#endif
#include "rclcpp/rclcpp.hpp"

using namespace industrial::joint_data;
using namespace industrial::shared_types;

namespace industrial
{
namespace joint_traj_pt_full
{

JointTrajPtFull::JointTrajPtFull(void)
{
  this->init();
}
JointTrajPtFull::~JointTrajPtFull(void)
{

}

void JointTrajPtFull::init()
{
  this->robot_id_ = 0;
  this->sequence_ = 0;
  this->valid_fields_ = 0;
  this->time_ = 0.0;
  this->positions_.init();
  this->velocities_.init();
  this->accelerations_.init();
}

void JointTrajPtFull::init(industrial::shared_types::shared_int robot_id,
          industrial::shared_types::shared_int sequence,
          industrial::shared_types::shared_int valid_fields,
          industrial::shared_types::shared_real time,
          industrial::joint_data::JointData & positions,
          industrial::joint_data::JointData & velocities,
          industrial::joint_data::JointData & accelerations)
{
  this->setRobotID(robot_id);
  this->setSequence(sequence);
  this->setTime(time);
  this->setPositions(positions);
  this->setVelocities(velocities);
  this->setAccelerations(accelerations);
  this->valid_fields_ = valid_fields;  // must happen after others are set
}

void JointTrajPtFull::copyFrom(JointTrajPtFull &src)
{
  this->setRobotID(src.getRobotID());
  this->setSequence(src.getSequence());
  src.getTime(this->time_);
  src.getPositions(this->positions_);
  src.getVelocities(this->velocities_);
  src.getAccelerations(this->accelerations_);
  this->valid_fields_ = src.valid_fields_;
}

bool JointTrajPtFull::operator==(JointTrajPtFull &rhs)
{
  return this->robot_id_ == rhs.robot_id_ &&
         this->sequence_ == rhs.sequence_ &&
         this->valid_fields_ == rhs.valid_fields_ &&
         ( !is_valid(ValidFieldTypes::TIME) || (this->time_ == rhs.time_) ) &&
         ( !is_valid(ValidFieldTypes::POSITION) || (this->positions_ == rhs.positions_) ) &&
         ( !is_valid(ValidFieldTypes::VELOCITY) || (this->velocities_ == rhs.velocities_) ) &&
         ( !is_valid(ValidFieldTypes::ACCELERATION) || (this->accelerations_ == rhs.accelerations_) );
}

bool JointTrajPtFull::load(industrial::byte_array::ByteArray *buffer)
{
  //RCLCPP_INFO(rclcpp::get_logger("joint_traj_pt_full"), "Executing joint trajectory point load");

  if (!buffer->load(this->robot_id_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_traj_pt_full"), "Failed to load joint traj pt. robot_id");
    return false;
  }

  if (!buffer->load(this->sequence_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_traj_pt_full"), "Failed to load joint traj. pt. sequence number");
    return false;
  }

  if (!buffer->load(this->valid_fields_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_traj_pt_full"), "Failed to load joint traj. pt. valid fields");
    return false;
  }

  if (!buffer->load(this->time_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_traj_pt_full"), "Failed to load joint traj. pt. time");
    return false;
  }

  if (!this->positions_.load(buffer))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_traj_pt_full"), "Failed to load joint traj. pt. positions");
    return false;
  }

  if (!this->velocities_.load(buffer))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_traj_pt_full"), "Failed to load joint traj. pt. velocities");
    return false;
  }

  if (!this->accelerations_.load(buffer))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_traj_pt_full"), "Failed to load joint traj. pt. accelerations");
    return false;
  }

  //RCLCPP_INFO(rclcpp::get_logger("joint_traj_pt_full"), "Trajectory point successfully loaded");
  return true;
}

bool JointTrajPtFull::unload(industrial::byte_array::ByteArray *buffer)
{
  //RCLCPP_INFO(rclcpp::get_logger("joint_traj_pt_full"), "Executing joint traj. pt. unload");

  if (!this->accelerations_.unload(buffer))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_traj_pt_full"), "Failed to unload joint traj. pt. accelerations");
    return false;
  }

  if (!this->velocities_.unload(buffer))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_traj_pt_full"), "Failed to unload joint traj. pt. velocities");
    return false;
  }

  if (!this->positions_.unload(buffer))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_traj_pt_full"), "Failed to unload joint traj. pt. positions");
    return false;
  }

  if (!buffer->unload(this->time_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_traj_pt_full"), "Failed to unload joint traj. pt. time");
    return false;
  }

  if (!buffer->unload(this->valid_fields_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_traj_pt_full"), "Failed to unload joint traj. pt. valid fields");
    return false;
  }

  if (!buffer->unload(this->sequence_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_traj_pt_full"), "Failed to unload joint traj. pt. sequence number");
    return false;
  }

  if (!buffer->unload(this->robot_id_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_traj_pt_full"), "Faild to unload joint traj. pt. robot_id");
    return false;
  }

  //RCLCPP_INFO(rclcpp::get_logger("joint_traj_pt_full"), "Joint traj. pt successfully unloaded");
  return true;
}

}
}

