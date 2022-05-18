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

#include <vector>
#include <string>

#include "simple_message/smpl_msg_connection.hpp"
#include "simple_message/message_manager.hpp"
#include "simple_message/message_handler.hpp"
#include "simple_message/socket/tcp_client.hpp"
#include "industrial_robot_client/joint_relay_handler.hpp"
#include "industrial_robot_client/robot_status_relay_handler.hpp"
#include "rclcpp/rclcpp.hpp"

namespace industrial_robot_client
{
namespace robot_state_interface
{

using industrial::smpl_msg_connection::SmplMsgConnection;
using industrial::message_manager::MessageManager;
using industrial::message_handler::MessageHandler;
using industrial::tcp_client::TcpClient;
using industrial_robot_client::joint_relay_handler::JointRelayHandler;
using industrial_robot_client::robot_status_relay_handler::RobotStatusRelayHandler;
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;

/**
 * @brief Generic template that reads state-data from a robot controller
 * and publishes matching messages to various ROS topics.
 *
 * Users should replace the default class members
 * to implement robot-specific behavior.
 */
//* RobotStateInterface
class RobotStateInterface : public rclcpp::Node
{

public:

  /**
   * @brief Default constructor.
   */
  RobotStateInterface();

  /**
   * @brief Initialize robot connection using default method.
   *
   * @param default_ip default IP address to use for robot connection [OPTIONAL]
   *                    - this value will be used if ROS param "robot_ip_address" cannot be read
   * @param default_port default port to use for robot connection [OPTIONAL]
   *                    - this value will be used if ROS param "~port" cannot be read
   *
   * @return true on success, false otherwise
   */
  bool init(std::string default_ip = "", int default_port = industrial::simple_socket::StandardSocketPorts::STATE);


  /**
   * @brief Initialize robot connection using specified method.
   *
   * @param connection new robot-connection instance (ALREADY INITIALIZED).
   *
   * @return true on success, false otherwise
   */
  bool init(industrial::smpl_msg_connection::SmplMsgConnection *connection);

  /**
   * @brief Initialize robot connection using specified method and joint-names.
   *
   * @param connection new robot-connection instance (ALREADY INITIALIZED).
   * @param joint_names list of joint-names for ROS topic
   *   - Count and order should match data sent to robot connection.
   *   - Use blank-name to skip (not publish) a joint-position
   *
   * @return true on success, false otherwise
   */
  bool init(industrial::smpl_msg_connection::SmplMsgConnection *connection, 
            std::vector<std::string>& joint_names);

  /**
   * @brief get current robot-connection instance.
   *
   * @return current robot connection object
   */
  industrial::smpl_msg_connection::SmplMsgConnection *get_connection()
  {
    return this->connection_;
  }

  /**
   * @brief get active message-manager object
   *
   * @return current message-manager object
   */
  industrial::message_manager::MessageManager *get_manager()
  {
    return &this->manager_;
  }

  std::vector<std::string> get_joint_names()
  {
    return this->joint_names_;
  }

  void run();

  /**
   * @brief Add a new handler.
   *
   * @param new message-handler for a specific msg-type (ALREADY INITIALIZED).
   * @param replace existing handler (of same msg-type), if exists
   */
  void add_handler(industrial::message_handler::MessageHandler *handler, 
                   bool allow_replace = true)
  {
    this->manager_.add(handler, allow_replace);
  }

protected:
  TcpClient default_tcp_connection_;
  JointRelayHandler default_joint_handler_;
  RobotStatusRelayHandler default_robot_status_handler_;

  SmplMsgConnection *connection_;
  MessageManager manager_;
  std::vector<std::string> joint_names_;

};

}//robot_state_interface
}//industrial_robot_cliet