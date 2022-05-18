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

#include "robot_middleware/connection_manager/tcp_server_manager.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstdio>

using namespace industrial::tcp_server;

namespace robot_middleware
{
namespace connection_manager
{

TcpServerManager::TcpServerManager(const std::string& name, int port) : SimpleSocketManager(name, port)
{
}

TcpServerManager::~TcpServerManager()
{
#ifndef NDEBUG
  printf("DEBUG: Destructing TcpServerManager '%s'\n", getName());
#endif
}

bool TcpServerManager::init()
{
  auto tcp_server = std::make_unique<TcpServer>();
  if (!tcp_server->init(port_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("tcp_server_manager"), "Failed to initialize connection '%s' (0.0.0.0:%d)", getName(), port_);
    return false;
  }

  conn_ = std::move(tcp_server);
  RCLCPP_INFO(rclcpp::get_logger("tcp_server_manager"), "Initialized connection '%s' (0.0.0.0:%d)", getName(), port_);

  return true;
}

bool TcpServerManager::connect()
{
  if (conn_->isConnected())
  {
    RCLCPP_WARN(rclcpp::get_logger("tcp_server_manager"), "[%s] Attempted to connect while in connected state", getName());
    return true;
  }

  if (connected_once_)
  {
    RCLCPP_INFO(rclcpp::get_logger("tcp_server_manager"), "[%s] Trying to re-establish connection", getName());
    // no init required for TcpServer!
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("tcp_server_manager"), "[%s] Trying to connect", getName());
  }

  while (rclcpp::ok())
  {
    // TcpServer::makeConnect blocks until client accepted!
    bool rtn = conn_->makeConnect();

#ifndef NDEBUG
    printf("DEBUG: makeConnect() returned %d for '%s'\n", rtn, getName());
#endif

    if (isConnected())
    {
      connected_once_ = true;
      RCLCPP_INFO(rclcpp::get_logger("tcp_server_manager"), LOGNAME, "[%s] Connected", getName());
      return true;
    }
  }

  return false;
}

}  // namespace connection_manager
}  // namespace robot_middleware