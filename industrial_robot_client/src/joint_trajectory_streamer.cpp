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

#include "industrial_robot_client/joint_trajectory_streamer.hpp"

using industrial::simple_message::SimpleMessage;

namespace industrial_robot_client
{
namespace joint_trajectory_streamer
{

JointTrajectoryStreamer::JointTrajectoryStreamer(int min_buffer_size) : min_buffer_size_(min_buffer_size), JointTrajectoryInterface()
{
};


bool JointTrajectoryStreamer::init(SmplMsgConnection* connection, const std::vector<std::string> &joint_names,
                                   const std::map<std::string, double> &velocity_limits)
{
  bool rtn = true;

  RCLCPP_INFO(this->get_logger(), "JointTrajectoryStreamer: init");

  rtn &= JointTrajectoryInterface::init(connection, joint_names, velocity_limits);

  this->mutex_.lock();
  this->current_point_ = 0;
  this->state_ = TransferStates::IDLE;
  this->streaming_thread_ =
      new boost::thread(boost::bind(&JointTrajectoryStreamer::streamingThread, this));
  RCLCPP_INFO(this->get_logger(), "Unlocking mutex");
  this->mutex_.unlock();

  return rtn;
}

JointTrajectoryStreamer::~JointTrajectoryStreamer()
{
  delete this->streaming_thread_;
}

void JointTrajectoryStreamer::jointTrajectoryCB(const trajectory_msgs::msg::JointTrajectory::SharedPtr &msg)
{
  RCLCPP_INFO(this->get_logger(), "Receiving joint trajectory message");

  // read current state value (should be atomic)
  const auto state = this->state_;

  RCLCPP_DEBUG(this->get_logger(), "Current state is: %d", state);

  // always request a stop of current trajectory execution if an empty trajectory
  // is received. We handle this separately from the check below, as the server
  // might be executing a trajectory which this client has already finished
  // uploading (due to buffering on the server side fi), and then our local state
  // would be "IDLE", and we'd end up not sending the stop request.
  if (msg->points.empty())
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Empty trajectory received while in state: " << TransferStates::to_string(state) << ". Canceling current trajectory.");
    this->mutex_.lock();
    trajectoryStop();
    this->mutex_.unlock();
    return;
  }

  // if we're currently streaming a trajectory and we're requested to stream another
  // we complain, as splicing is not supported. Cancellation of the current trajectory
  // should first be requested, then a new trajectory started.
  if (TransferStates::IDLE != state)
  {
    RCLCPP_ERROR(this->get_logger(), "Trajectory splicing not yet implemented, stopping current motion.");
    this->mutex_.lock();
    trajectoryStop();
    this->mutex_.unlock();
    return;
  }

  // calc new trajectory
  std::vector<JointTrajPtMessage> new_traj_msgs;
  if (!trajectory_to_msgs(msg, &new_traj_msgs))
    return;

  // send command messages to robot
  send_to_robot(new_traj_msgs);
}

bool JointTrajectoryStreamer::send_to_robot(const std::vector<JointTrajPtMessage>& messages)
{
  RCLCPP_INFO(this->get_logger(), "Loading trajectory, setting state to streaming");
  this->mutex_.lock();
  {
    RCLCPP_INFO(this->get_logger(), "Executing trajectory of size: %d", (int)messages.size());
    this->current_traj_ = messages;
    this->current_point_ = 0;
    this->state_ = TransferStates::STREAMING;
    this->streaming_start_ = rclcpp::Clock(RCL_ROS_TIME).now();
  }
  this->mutex_.unlock();

  return true;
}

bool JointTrajectoryStreamer::trajectory_to_msgs(const trajectory_msgs::msg::JointTrajectory::SharedPtr &traj, std::vector<JointTrajPtMessage>* msgs)
{
  // use base function to transform points
  if (!JointTrajectoryInterface::trajectory_to_msgs(traj, msgs))
    return false;

  // pad trajectory as required for minimum streaming buffer size
  if (!msgs->empty() && (msgs->size() < (size_t)min_buffer_size_))
  {
    RCLCPP_DEBUG(this->get_logger(), "Padding trajectory: current(%d) => minimum(%d)", (int)msgs->size(), min_buffer_size_);
    while (msgs->size() < (size_t)min_buffer_size_)
      msgs->push_back(msgs->back());
  }

  return true;
}


void JointTrajectoryStreamer::streamingThread()
{
  JointTrajPtMessage jtpMsg;
  int connectRetryCount = 1;

  RCLCPP_INFO(this->get_logger(), "Starting joint trajectory streamer thread");
  while (rclcpp::ok())
  {
    rclcpp::Rate(1/0.005).sleep();
    // automatically re-establish connection, if required
    if (connectRetryCount-- > 0)
    {
      RCLCPP_INFO(this->get_logger(), "Connecting to robot motion server");
      this->connection_->makeConnect();
      rclcpp::Rate(1/0.250).sleep();// wait for connection


      if (this->connection_->isConnected())
        connectRetryCount = 0;
      else if (connectRetryCount <= 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Timeout connecting to robot controller.  Send new motion command to retry.");
        this->state_ = TransferStates::IDLE;
      }
      continue;
    }

    this->mutex_.lock();

    SimpleMessage msg, reply;
        
    switch (this->state_)
    {
      case TransferStates::IDLE:
        rclcpp::Rate(1/0.010).sleep();  //  loop while waiting for new trajectory
        break;

      case TransferStates::STREAMING:
        if (this->current_point_ >= (int)this->current_traj_.size())
        {
          RCLCPP_INFO(this->get_logger(), "Trajectory streaming complete, setting state to IDLE");
          this->state_ = TransferStates::IDLE;
          break;
        }

        if (!this->connection_->isConnected())
        {
          RCLCPP_DEBUG(this->get_logger(), "Robot disconnected.  Attempting reconnect...");
          connectRetryCount = 5;
          break;
        }

        jtpMsg = this->current_traj_[this->current_point_];
        jtpMsg.toRequest(msg);
            
        RCLCPP_DEBUG(this->get_logger(), "Sending joint trajectory point");
        if (this->connection_->sendAndReceiveMsg(msg, reply, false))
        {
          this->current_point_++;
          RCLCPP_INFO(this->get_logger(), "Point[%d of %d] sent to controller",
                   this->current_point_, (int)this->current_traj_.size());
        }
        else
          RCLCPP_WARN(this->get_logger(), "Failed sent joint point, will try again");

        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Joint trajectory streamer: unknown state");
        this->state_ = TransferStates::IDLE;
        break;
    }

    this->mutex_.unlock();
  }

  RCLCPP_WARN(this->get_logger(), "Exiting trajectory streamer thread");
}

void JointTrajectoryStreamer::trajectoryStop()
{
  JointTrajectoryInterface::trajectoryStop();

  RCLCPP_DEBUG(this->get_logger(), "Stop command sent, entering idle mode");
  this->state_ = TransferStates::IDLE;
}

} //joint_trajectory_streamer
} //industrial_robot_client

