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

#include "rclcpp/rclcpp.hpp"

#include <string>

namespace robot_middleware
{

double toRadians(double angle_deg);

struct CartesianLimits
{

  double max_linear_velocity;
  double max_angular_velocity;
};

struct JointLimits
{
  double toRadians(double angle_deg);

  bool has_velocity_limit;
  double max_velocity;
  double stop_tolerance;
};

struct VelocityControlSettings
{
  explicit VelocityControlSettings(std::shared_ptr<rclcpp::Node> node);
  ~VelocityControlSettings();

  bool initParam();

  std::shared_ptr<rclcpp::Node> node_;
  std::string base_frame;
  std::string tool_frame;
  double control_loop_frequency;
  CartesianLimits cartesian_limits;
  JointLimits joint_limits;

private:
  const std::string LOGNAME;
};

}  // namespace robot_middleware