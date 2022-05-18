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
#include "robot_middleware/velocity_control_settings.hpp"

#include "control_msgs/msg/joint_jog.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "motion_control_msgs/msg/velocity_command.hpp"
#include "rclcpp/rclcpp.hpp"

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace robot_middleware
{

enum class JogInterfaceState
{
  IDLE,
  CARTESIAN_JOGGING,
  JOINT_JOGGING
};

class JogInterface 
{
public:
  explicit JogInterface(std::shared_ptr<rclcpp::Node> node);

  ~JogInterface();

  void init(const std::shared_ptr<RobotDriver>& driver);

  void start();

  void stop();

  void reset();

private:
  void twistCb(const geometry_msgs::msg::TwistStamped::SharedPtr cmd);

  void jointJogCb(const control_msgs::msg::JointJog::SharedPtr cmd);

  bool inError(rclcpp::Time latest_cmd_timestamp);

  bool setCommand(const geometry_msgs::msg::TwistStamped::ConstPtr& cmd);

  bool setCommand(const control_msgs::msg::JointJog::ConstPtr& cmd);

  bool setState(JogInterfaceState new_state);

  void loop();

  bool controlLoop();

  const std::string LOGNAME = "jog_interface";
  const double DEFAULT_RECEIVE_TIMEOUT = 0.1;  // seconds

  std::shared_ptr<RobotDriver> driver_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_;
  rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr sub_joint_jog_;
  std::shared_ptr<rclcpp::Node> node_;
  geometry_msgs::msg::Twist twist_cmd_;
  control_msgs::msg::JointJog joint_jog_cmd_;
  motion_control_msgs::msg::VelocityCommand vel_cmd_;
  rclcpp::Time cmd_timestamp_;

  JogInterfaceState state_;
  double control_loop_hz_;
  double receive_timeout_;
  bool keep_running_;
  bool in_error_;

  std::thread control_task_;
  std::condition_variable command_received_cv_;
  std::mutex state_mtx_;
};

}  // namespace robot_middleware