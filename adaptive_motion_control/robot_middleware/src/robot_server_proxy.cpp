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

#include "robot_middleware/robot_server_proxy.hpp"

using namespace robot_middleware::connection_manager;

namespace robot_middleware
{

RobotServerProxy::RobotServerProxy(std::shared_ptr<rclcpp::Node> node)
{
  node_ = node;
}

RobotServerProxy::~RobotServerProxy()
{
}

bool RobotServerProxy::init(int default_motion_port, int default_state_port)
{
  int motion_port, state_port;

  node_->declare_parameter<int>("robot_server_proxy_motion_server_port", default_motion_port);
  node_->declare_parameter<int>("robot_server_proxy_state_server_port", default_state_port);
  node_->get_parameter<int>("robot_server_proxy_motion_server_port", motion_port);
  node_->get_parameter<int>("robot_server_proxy_state_server_port", state_port);

  RCLCPP_INFO(node_->get_logger(), "Using motion server port: %d", motion_port);
  RCLCPP_INFO(node_->get_logger(), "Using state server port: %d", state_port);

  auto motion_server_manager = std::make_shared<TcpServerManager>("motion_server", motion_port);
  auto state_server_manager = std::make_shared<TcpServerManager>("state_server", state_port);

  if (!motion_server_manager->init())
    return false;

  if (!state_server_manager->init())
    return false;

  motion_server_manager_ = std::move(motion_server_manager);
  state_server_manager_ = std::move(state_server_manager);

  return true;
}

void RobotServerProxy::connect()
{
  motion_server_manager_->startConnectionTask();
  state_server_manager_->startConnectionTask();
}

}  // namespace robot_middleware
