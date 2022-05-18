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

#include "robot_middleware/connection_manager/tcp_client_manager.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstdio>

using namespace industrial::tcp_client;

namespace robot_middleware
{
namespace connection_manager
{

TcpClientManager::TcpClientManager(const std::string& name, const std::string& ip_address, int port)
  : SimpleSocketManager(name, port), ip_address_(strdup(ip_address.c_str()))
{
}

TcpClientManager::~TcpClientManager()
{
#ifndef NDEBUG
  printf("DEBUG: Destructing TcpClientManager '%s'\n", getName());
#endif
  free(ip_address_);
}

bool TcpClientManager::init()
{
  auto tcp_client = std::make_unique<TcpClient>();
  if (!tcp_client->init(ip_address_, port_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("tcp_client_manager"), "Failed to initialize connection '%s' (%s:%d)", getName(), ip_address_, port_);
    return false;
  }

  conn_ = std::move(tcp_client);
  RCLCPP_INFO(rclcpp::get_logger("tcp_client_manager"), "Initialized connection '%s' (%s:%d)", getName(), ip_address_, port_);

  return true;
}

bool TcpClientManager::connect()
{
  using namespace std::chrono_literals;

  if (conn_->isConnected())
  {
    RCLCPP_WARN(rclcpp::get_logger("tcp_client_manager"), "[%s] Attempted to connect while in connected state", getName());
    return true;
  }

  if (connected_once_)
  {
    RCLCPP_INFO(rclcpp::get_logger("tcp_client_manager"), "[%s] Trying to re-establish connection", getName());
    init();
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("tcp_client_manager"), "[%s] Trying to connect", getName());
  }

  while (rclcpp::ok())
  {
    if (conn_->makeConnect())
    {
      std::this_thread::sleep_for(250ms);
      if (conn_->isConnected())
      {
        connected_once_ = true;
        RCLCPP_INFO(rclcpp::get_logger("tcp_client_manager"), "Connected");
        return true;
      }
    }

    // throttle loop if connection fails
    int seconds = CONNECT_RETRY_DELAY;
    while (rclcpp::ok() && seconds > 0)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("tcp_client_manager"), "[%s] Trying to connect again in %d seconds", getName(), seconds);
      std::this_thread::sleep_for(1s);
      seconds--;
    }
  }

  return false;
}

}  // namespace connection_manager
}  // namespace robot_middleware