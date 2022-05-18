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

#include "robot_middleware/velocity_control_settings.hpp"

#include "rclcpp/rclcpp.hpp"

namespace robot_middleware
{

double toRadians(double angle_degrees)
{
  double angle_rad = angle_degrees * (M_PI/180);
  return angle_rad;
}

VelocityControlSettings::VelocityControlSettings(std::shared_ptr<rclcpp::Node> node)
{
  node_ = node;
}

VelocityControlSettings::~VelocityControlSettings()
{
}

bool VelocityControlSettings::initParam()
{
  std::size_t error = 0;

  node_->declare_parameter<std::string>("velocity_control_base_frame", "base");
  node_->declare_parameter<std::string>("velocity_control_tool_frame", "tool0");
  node_->declare_parameter<double>("velocity_control_control_loop_frequency", 250.0);
  node_->declare_parameter<double>("velocity_control_max_linear_velocity", 0.250);
  node_->declare_parameter<double>("velocity_control_max_angular_velocity", toRadians(60.0));
  node_->declare_parameter<bool>("velocity_control_has_velocity_limit", true);
  node_->declare_parameter<double>("velocity_control_max_velocity", toRadians(60.0));
  node_->declare_parameter<double>("velocity_control_stop_tolerance", toRadians(3.0));

  node_->get_parameter<std::string>("velocity_control_base_frame", base_frame);
  node_->get_parameter<std::string>("velocity_control_tool_frame", tool_frame);
  node_->get_parameter<double>("velocity_control_control_loop_frequency", control_loop_frequency);
  node_->get_parameter<double>("velocity_control_max_linear_velocity", cartesian_limits.max_linear_velocity);
  node_->get_parameter<double>("velocity_control_max_angular_velocity", cartesian_limits.max_angular_velocity);
  node_->get_parameter<bool>("velocity_control_has_velocity_limit", joint_limits.has_velocity_limit);
  node_->get_parameter<double>("velocity_control_max_velocity", joint_limits.max_velocity);
  node_->get_parameter<double>("velocity_control_stop_tolerance", joint_limits.stop_tolerance);

  return true;
}

}  // namespace robot_middleware
