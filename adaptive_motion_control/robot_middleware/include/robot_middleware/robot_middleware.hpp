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

#include "robot_middleware/jog_interface.hpp"
#include "robot_middleware/message_relay_handler/motion_relay_handler.hpp"
#include "robot_middleware/message_relay_handler/state_relay_handler.hpp"
#include "robot_middleware/pose_tracking_controller.hpp"
#include "robot_middleware/robot_driver.hpp"
#include "robot_middleware/robot_server_proxy.hpp"
#include "robot_middleware/staubli/staubli_driver.hpp"
#include "robot_middleware/velocity_control_settings.hpp"

#include "moveit/robot_model_loader/robot_model_loader.h"
#include "rclcpp/rclcpp.hpp"
#include "simple_message/socket/tcp_server.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <thread>

namespace robot_middleware
{

class RobotMiddleware : public rclcpp::Node
{
public:
  explicit RobotMiddleware();

  ~RobotMiddleware();

  bool init();

  void run();

private:

  std::shared_ptr<RobotDriver> driver_;
  std::shared_ptr<RobotServerProxy> server_proxy_;
  std::shared_ptr<JogInterface> jog_interface_;
  std::shared_ptr<PoseTrackingController> pose_tracking_controller_;
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  std::shared_ptr<robot_middleware::message_relay_handler::StateRelayHandler> state_relay_handler_;
  std::shared_ptr<robot_middleware::message_relay_handler::MotionRelayHandler> motion_relay_handler_;
};

}  // namespace robot_middleware