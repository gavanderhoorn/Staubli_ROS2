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
#include "simple_message/robot_status.hpp"
#include "simple_message/shared_types.hpp"

#else
#include "robot_status.hpp"
#include "shared_types.hpp"
#endif
#include "rclcpp/rclcpp.hpp"

// remove ROS after Melodic (bw compat for #262)
#if defined(SIMPLE_MESSAGE_USE_ROS) || defined(ROS)
// Files below used to translate between ROS messages enums and
// enums defined in this file
#include "industrial_msgs/msg/robot_mode.hpp"
#include "industrial_msgs/msg/tri_state.hpp"
#endif

using namespace industrial::shared_types;

namespace industrial
{
namespace robot_status
{

namespace RobotModes
{

// remove ROS after Melodic (bw compat for #262)
#if defined(SIMPLE_MESSAGE_USE_ROS) || defined(ROS)

int toROSMsgEnum(RobotModes::RobotMode mode)
{

  switch (mode)
  {
    case RobotModes::AUTO:
      return industrial_msgs::msg::RobotMode::AUTO;
      break;
    case RobotModes::MANUAL:
      return industrial_msgs::msg::RobotMode::MANUAL;
      break;
    case RobotModes::UNKNOWN:
      return industrial_msgs::msg::RobotMode::UNKNOWN;
  }
  return industrial_msgs::msg::RobotMode::UNKNOWN;

}
;

#endif

}

namespace TriStates
{

// remove ROS after Melodic (bw compat for #262)
#if defined(SIMPLE_MESSAGE_USE_ROS) || defined(ROS)

int toROSMsgEnum(TriStates::TriState state)
{

  switch (state)
  {
    case TriStates::TS_UNKNOWN:
      return industrial_msgs::msg::TriState::UNKNOWN;
      break;
    case TriStates::TS_TRUE:
      return industrial_msgs::msg::TriState::TRUE;
      break;
    case TriStates::TS_FALSE:
      return industrial_msgs::msg::TriState::FALSE;
      break;
  }
  return industrial_msgs::msg::TriState::UNKNOWN;

}
;

#endif

}

RobotStatus::RobotStatus(void)
{
  this->init();
}
RobotStatus::~RobotStatus(void)
{

}

void RobotStatus::init()
{
  this->init(TriStates::TS_UNKNOWN, TriStates::TS_UNKNOWN, 0, TriStates::TS_UNKNOWN,
             TriStates::TS_UNKNOWN, RobotModes::UNKNOWN, TriStates::TS_UNKNOWN);
}

void RobotStatus::init(TriState drivesPowered, TriState eStopped, industrial::shared_types::shared_int errorCode,
                       TriState inError, TriState inMotion, RobotMode mode, TriState motionPossible)
{
  this->setDrivesPowered(drivesPowered);
  this->setEStopped(eStopped);
  this->setErrorCode(errorCode);
  this->setInError(inError);
  this->setInMotion(inMotion);
  this->setMode(mode);
  this->setMotionPossible(motionPossible);
}

void RobotStatus::copyFrom(RobotStatus &src)
{
  this->setDrivesPowered(src.getDrivesPowered());
  this->setEStopped(src.getEStopped());
  this->setErrorCode(src.getErrorCode());
  this->setInError(src.getInError());
  this->setInMotion(src.getInMotion());
  this->setMode(src.getMode());
  this->setMotionPossible(src.getMotionPossible());
}

bool RobotStatus::operator==(RobotStatus &rhs)
{
  return this->drives_powered_ == rhs.drives_powered_ && this->e_stopped_ == rhs.e_stopped_
      && this->error_code_ == rhs.error_code_ && this->in_error_ == rhs.in_error_ && this->in_motion_ == rhs.in_motion_
      && this->mode_ == rhs.mode_ && this->motion_possible_ == rhs.motion_possible_;
}

bool RobotStatus::load(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;

  //RCLCPP_INFO(rclcpp::get_logger("robot_status"), "Executing robot status load");

  if (buffer->load(this->drives_powered_) && buffer->load(this->e_stopped_) && buffer->load(this->error_code_)
      && buffer->load(this->in_error_) && buffer->load(this->in_motion_) && buffer->load(this->mode_)
      && buffer->load(this->motion_possible_))
  {

    //RCLCPP_INFO(rclcpp::get_logger("robot_status"), "Robot status successfully loaded");
    rtn = true;
  }
  else
  {
    //RCLCPP_INFO(rclcpp::get_logger("robot_status"), "Robot status not loaded");
    rtn = false;
  }

  return rtn;
}

bool RobotStatus::unload(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;

  //RCLCPP_INFO(rclcpp::get_logger("robot_status"), "Executing robot status unload");
  if (buffer->unload(this->motion_possible_) && buffer->unload(this->mode_) && buffer->unload(this->in_motion_)
      && buffer->unload(this->in_error_) && buffer->unload(this->error_code_) && buffer->unload(this->e_stopped_)
      && buffer->unload(this->drives_powered_))
  {

    rtn = true;
    //RCLCPP_INFO(rclcpp::get_logger("robot_status"), "Robot status successfully unloaded");
  }

  else
  {
    //RCLCPP_ERROR(rclcpp::get_logger("robot_status"), "Failed to unload robot status");
    rtn = false;
  }

  return rtn;
}

}
}

