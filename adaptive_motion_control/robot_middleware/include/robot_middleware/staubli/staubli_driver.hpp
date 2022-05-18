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

#pragma once

#include "robot_middleware/robot_driver.hpp"
#include "simple_message/velocity_command_type.hpp"

#include "motion_control_msgs/msg/velocity_command.hpp"
#include "moveit/robot_model/robot_model.h"
#include "rclcpp/rclcpp.hpp"

#include <string>

namespace staubli
{

class StaubliDriver : public robot_middleware::RobotDriver
{
public:
  explicit StaubliDriver(const std::string& robot_ip, const moveit::core::RobotModelConstPtr& robot_model, std::shared_ptr<rclcpp::Node> node);

  ~StaubliDriver();

  bool sendVelocityCommand_internal(const motion_control_msgs::msg::VelocityCommand& vel_cmd) override;

  bool sendVelocityConfig_internal(uint8_t type) override;

private:
  industrial::velocity_command_type::VelocityCommandType toStaubliVelocityCommandType(uint8_t type);
};

}  // namespace staubli
