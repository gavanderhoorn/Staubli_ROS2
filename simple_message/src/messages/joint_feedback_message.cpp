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

#ifndef FLATHEADERS
#include "simple_message/messages/joint_feedback_message.hpp"
#include "simple_message/joint_data.hpp"
#include "simple_message/byte_array.hpp"
#else
#include "joint_feedback_message.hpp"
#include "joint_data.hpp"
#include "byte_array.hpp"
#endif

#include "rclcpp/rclcpp.hpp"


using namespace industrial::shared_types;
using namespace industrial::byte_array;
using namespace industrial::simple_message;
using namespace industrial::joint_feedback;

namespace industrial
{
namespace joint_feedback_message
{

JointFeedbackMessage::JointFeedbackMessage(void)
{
  this->init();
}

JointFeedbackMessage::~JointFeedbackMessage(void)
{

}

bool JointFeedbackMessage::init(industrial::simple_message::SimpleMessage & msg)
{
  bool rtn = false;
  ByteArray data = msg.getData();
  this->init();

  if (data.unload(this->data_))
  {
    rtn = true;
  }
  else
  {
    // LOG_ERROR("Failed to unload joint feedback message data");
  }
  return rtn;
}

void JointFeedbackMessage::init(industrial::joint_feedback::JointFeedback & data)
{
  this->init();
  this->data_.copyFrom(data);
}

void JointFeedbackMessage::init()
{
  this->setMessageType(StandardMsgTypes::JOINT_FEEDBACK);
  this->data_.init();
}

bool JointFeedbackMessage::load(ByteArray *buffer)
{
  bool rtn = false;
  // LOG_COMM("Executing joint feedback message load");
  if (buffer->load(this->data_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    //RCLCPP_ERROR(rclcpp::get_logger("joint_feedback_message"), "Failed to load joint feedback message data");
  }
  return rtn;
}

bool JointFeedbackMessage::unload(ByteArray *buffer)
{
  bool rtn = false;
  //RCLCPP_INFO(rclcpp::get_logger("joint_feedback_message"), "Executing joint feedback message unload");

  if (buffer->unload(this->data_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    //RCLCPP_ERROR(rclcpp::get_logger("joint_feedback_message"), "Failed to unload joint feedback message data");
  }
  return rtn;
}

}
}

