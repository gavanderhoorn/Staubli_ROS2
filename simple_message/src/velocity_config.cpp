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

#include "simple_message/velocity_config.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstring>

using namespace industrial::byte_array;
using namespace industrial::shared_types;

namespace industrial
{
namespace velocity_config
{

VelocityConfig::VelocityConfig()
{
  this->init();
}

VelocityConfig::~VelocityConfig()
{
}

void VelocityConfig::init()
{
  this->cmd_type_ = 0;
  std::memset(this->frame_ref_, 0, sizeof(this->frame_ref_));
  std::memset(this->tool_ref_, 0, sizeof(this->tool_ref_));
  this->accel_ = 0.0;
  this->vel_ = 0.0;
  this->tvel_ = 0.0;
  this->rvel_ = 0.0;
}

bool VelocityConfig::load(ByteArray* buffer)
{
  //RCLCPP_INFO(rclcpp::get_logger("velocity_config"), "Executing velocity config load");

  if (!buffer->load(this->cmd_type_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("velocity_config"), "Failed to load velocity config cmd_type");
    return false;
  }

  for (const shared_real& value : this->frame_ref_)
  {
    if (!buffer->load(value))
    {
      //RCLCPP_ERROR(rclcpp::get_logger("velocity_config"), "Failed to load velocity config frame_ref");
      return false;
    }
  }

  for (const shared_real& value : this->tool_ref_)
  {
    if (!buffer->load(value))
    {
      //RCLCPP_ERROR(rclcpp::get_logger("velocity_config"), "Failed to load velocity config tool_ref");
      return false;
    }
  }

  if (!buffer->load(accel_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("velocity_config"), "Failed to load velocity config accel");
    return false;
  }

  if (!buffer->load(vel_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("velocity_config"), "Failed to load velocity config vel");
    return false;
  }

  if (!buffer->load(this->tvel_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("velocity_config"), "Failed to load velocity config tvel");
    return false;
  }

  if (!buffer->load(this->rvel_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("velocity_config"), "Failed to load velocity config rvel");
    return false;
  }

  return true;
}

bool VelocityConfig::unload(ByteArray* buffer)
{
  //RCLCPP_INFO(rclcpp::get_logger("velocity_config"), "Executing velocity config unload");

  if (!buffer->unload(this->rvel_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("velocity_config"), "Failed to unload velocity config rvel");
    return false;
  }

  if (!buffer->unload(this->tvel_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("velocity_config"), "Failed to unload velocity config tvel");
    return false;
  }

  if (!buffer->unload(this->vel_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("velocity_config"), "Failed to unload velocity config vel");
    return false;
  }

  if (!buffer->unload(this->accel_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("velocity_config"), "Failed to unload velocity config accel");
    return false;
  }

  for (int i = 5; i >= 0; i--)
  {
    if (!buffer->unload(this->tool_ref_[i]))
    {
      //RCLCPP_ERROR(rclcpp::get_logger("velocity_config"), "Failed to unload velocity config tool_ref");
      return false;
    }
  }

  for (int i = 5; i >= 0; i--)
  {
    if (!buffer->unload(this->frame_ref_[i]))
    {
      //RCLCPP_ERROR(rclcpp::get_logger("velocity_config"), "Failed to unload velocity config frame_ref");
      return false;
    }
  }

  if (!buffer->unload(this->cmd_type_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("velocity_config"), "Failed to unload velocity config cmd_type");
    return false;
  }

  return true;
}

}  // namespace industrial
}  // namespace velocity_config