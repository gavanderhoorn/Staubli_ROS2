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

#include "industrial_robot_client/joint_relay_handler.hpp"
#include "rcpputils/asserts.hpp"

using industrial::shared_types::shared_real;
using industrial::smpl_msg_connection::SmplMsgConnection;
using namespace industrial::simple_message;

namespace industrial_robot_client
{
namespace joint_relay_handler
{

JointRelayHandler::JointRelayHandler() : Node("joint_relay_handler")
{
};

bool JointRelayHandler::init(industrial::smpl_msg_connection::SmplMsgConnection *connection, 
                             std::vector<std::string>& joint_names)
{
  pub_joint_control_state_ = this->create_publisher<control_msgs::action::FollowJointTrajectory_FeedbackMessage>("feedback_states", 1);
  pub_joint_sensor_state_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",1);

  // save "complete" joint-name list, preserving any blank entries for later use
  all_joint_names_ = joint_names;

  return init((int)industrial::simple_message::StandardMsgTypes::JOINT, connection);
}

bool JointRelayHandler::internalCB(industrial::simple_message::SimpleMessage &in)
{
  industrial::joint_message::JointMessage joint_msg;

  if (!joint_msg.init(in))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize joint message");
    return false;
  }

  return internalCB(joint_msg);
}

bool JointRelayHandler::internalCB(industrial::joint_message::JointMessage &in)
{
  control_msgs::action::FollowJointTrajectory_FeedbackMessage control_state;
  sensor_msgs::msg::JointState sensor_state;
  bool rtn = true;

  if (create_messages(in, &control_state, &sensor_state))
  {
    pub_joint_control_state_->publish(control_state);
    pub_joint_sensor_state_->publish(sensor_state);
  }
  else
    rtn = false;

  // Reply back to the controller if the sender requested it.
  if (industrial::simple_message::CommTypes::SERVICE_REQUEST == in.getMessageType())
  {
    industrial::simple_message::SimpleMessage reply;
    in.toReply(reply, rtn ? industrial::simple_message::ReplyTypes::SUCCESS : industrial::simple_message::ReplyTypes::FAILURE);
    this->getConnection()->sendMsg(reply);
  }

  return rtn;
}

// TODO: Add support for other message fields (velocity, effort, desired pos)
bool JointRelayHandler::create_messages(industrial::joint_message::JointMessage &msg_in,
                                        control_msgs::action::FollowJointTrajectory_FeedbackMessage *control_state,
                                        sensor_msgs::msg::JointState *sensor_state)
{
  // read joint positions from JointMessage
  std::vector<double> all_joint_pos(all_joint_names_.size());
  for (int i=0; i<all_joint_names_.size(); ++i)
  {
    industrial::shared_types::shared_real value;
    if (msg_in.getJoints().getJoint(i, value))
      all_joint_pos[i] = value;
    else
      RCLCPP_ERROR(this->get_logger(), "Failed to parse #%d value from JointMessage", i);
  }

  // apply transform to joint positions, if required
  std::vector<double> xform_joint_pos;
  if (!transform(all_joint_pos, &xform_joint_pos))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to transform joint positions");
    return false;
  }

  // select specific joints for publishing
  std::vector<double> pub_joint_pos;
  std::vector<std::string> pub_joint_names;
  if (!select(xform_joint_pos, all_joint_names_, &pub_joint_pos, &pub_joint_names))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to select joints for publishing");
    return false;
  }

  // assign values to messages
  control_msgs::action::FollowJointTrajectory_FeedbackMessage tmp_control_state;  // always start with a "clean" message
  tmp_control_state.feedback.header.stamp = this->now();
  tmp_control_state.feedback.joint_names = pub_joint_names;
  tmp_control_state.feedback.actual.positions = pub_joint_pos;
  *control_state = tmp_control_state;

  sensor_msgs::msg::JointState tmp_sensor_state;
  tmp_sensor_state.header.stamp = this->now();
  tmp_sensor_state.name = pub_joint_names;
  tmp_sensor_state.position = pub_joint_pos;
  *sensor_state = tmp_sensor_state;

  return true;
}

bool JointRelayHandler::select(const std::vector<double>& all_joint_pos, 
                               const std::vector<std::string>& all_joint_names,
                               std::vector<double>* pub_joint_pos, 
                               std::vector<std::string> *pub_joint_names)
{
  rcpputils::assert_true(all_joint_pos.size() == all_joint_names.size());

  pub_joint_pos->clear();
  pub_joint_names->clear();

  // skip over "blank" joint names
  for (int i=0; i<all_joint_pos.size(); ++i)
  {
    if (all_joint_names[i].empty())
      continue;

    pub_joint_pos->push_back(all_joint_pos[i]);
    pub_joint_names->push_back(all_joint_names[i]);
  }
  return true;
}

}//namespace joint_relay_handler
}//namespace industrial_robot_client