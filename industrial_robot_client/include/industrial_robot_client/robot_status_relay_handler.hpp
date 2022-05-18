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
#include "simple_message/message_handler.hpp"
#include "simple_message/messages/robot_status_message.hpp"
#include "industrial_msgs/msg/robot_status.hpp"

namespace industrial_robot_client
{
namespace robot_status_relay_handler
{

/**
 * @brief Message handler that relays joint positions (converts simple message
 * types to ROS message types and publishes them)
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class RobotStatusRelayHandler : public industrial::message_handler::MessageHandler, public rclcpp::Node
{
  // since this class defines a different init(), this helps find the base-class init()
  using industrial::message_handler::MessageHandler::init;

public:

  /**
* @brief Constructor
*/
  RobotStatusRelayHandler();


 /**
  * @brief Class initializer
  *
  * @param connection simple message connection that will be used to send replies.
  *
  * @return true on success, false otherwise (an invalid message type)
  */
 bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection);

protected:

  rclcpp::Publisher<industrial_msgs::msg::RobotStatus>::SharedPtr pub_robot_status_;

  /**
   * @brief Callback executed upon receiving a robot status message
   *
   * @param in incoming message
   *
   * @return true on success, false otherwise
   */
  bool internalCB(industrial::robot_status_message::RobotStatusMessage & in);

private:
 /**
  * @brief Callback executed upon receiving a message
  *
  * @param in incoming message
  *
  * @return true on success, false otherwise
  */
 bool internalCB(industrial::simple_message::SimpleMessage& in);

};

}
}
