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

#include "industrial_robot_client/robot_state_interface.hpp"
#include "industrial_utils/param_utils.hpp"

using industrial::smpl_msg_connection::SmplMsgConnection;
using industrial_utils::param::ParamUtils;
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;

namespace industrial_robot_client
{
namespace robot_state_interface
{

RobotStateInterface::RobotStateInterface() : Node("robot_state_interface")
{ 
  this->connection_ = NULL;
  this->add_handler(&default_joint_handler_);
  this->add_handler(&default_robot_status_handler_);
}

bool RobotStateInterface::init(std::string default_ip, int default_port)
{
  std::string ip;
  int port;

  // override IP/port with ROS params, if available
  this->declare_parameter<std::string>("robot_ip_address", default_ip);
  this->declare_parameter<int>("~port", default_port);
  this->get_parameter("robot_ip_address", ip);
  this->get_parameter("~port", port);

  // check for valid parameter values
  if (ip.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "No valid robot IP address found.  Please set ROS 'robot_ip_address' param");
    return false;
  }
  if (port <= 0)
  {
    RCLCPP_ERROR(this->get_logger(), "No valid robot IP port found.  Please set ROS '~port' param");
    return false;
  }

  char* ip_addr = strdup(ip.c_str());  // connection.init() requires "char*", not "const char*"
  RCLCPP_INFO(this->get_logger(), "Robot state connecting to IP address: '%s:%d'", ip_addr, port);
  default_tcp_connection_.init(ip_addr, port);
  free(ip_addr);

  return init(&default_tcp_connection_);
}

bool RobotStateInterface::init(industrial::smpl_msg_connection::SmplMsgConnection *connection)
{
  std::vector<std::string> joint_names;
  ParamUtils pu;
  if (!pu.getJointNames("move_group", "rviz2", "controller_joint_names", "robot_description", joint_names))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize joint_names.  Aborting");
    return false;
  }

  return init(connection, joint_names);
}

bool RobotStateInterface::init(industrial::smpl_msg_connection::SmplMsgConnection *connection, 
                               std::vector<std::string>& joint_names)
{
  this->joint_names_ = joint_names;
  this->connection_ = connection;
  connection_->makeConnect();

  // initialize message-manager
  if (!manager_.init(connection_))
    return false;

  // initialize default handlers
  if (!default_joint_handler_.init(connection_, joint_names_))
    return false;
  this->add_handler(&default_joint_handler_);

  if (!default_robot_status_handler_.init(connection_))
      return false;
  this->add_handler(&default_robot_status_handler_);

  return true;
}

void RobotStateInterface::run()
{
  manager_.spin();
}

} // robot_state_interface
} // industrial_robot_client