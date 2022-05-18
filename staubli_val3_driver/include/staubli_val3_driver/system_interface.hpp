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

#include <string>

#include "set_drive_power_message.hpp"
#include "industrial_msgs/srv/set_drive_power.hpp"
#include "simple_message/socket/simple_socket.hpp"
#include "simple_message/smpl_msg_connection.hpp"

class SystemInterface : public rclcpp::Node
{
public:
  SystemInterface();

  ~SystemInterface();

  bool init(const std::string& default_ip = "",
            int default_port = industrial::simple_socket::StandardSocketPorts::SYSTEM);

  void run();
  
  void setDrivePowerCb(const std::shared_ptr<industrial_msgs::srv::SetDrivePower::Request> req,
                       std::shared_ptr<industrial_msgs::srv::SetDrivePower::Response> res);

  bool setDrivePower(bool value);

private:
  std::shared_ptr<industrial::smpl_msg_connection::SmplMsgConnection> connection_;
  rclcpp::Service<industrial_msgs::srv::SetDrivePower>::SharedPtr service;
};