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

#include "industrial_robot_client/joint_trajectory_downloader.hpp"

namespace industrial_robot_client
{
namespace joint_trajectory_downloader
{

using industrial::simple_message::SimpleMessage;
namespace SpecialSeqValues = industrial::joint_traj_pt::SpecialSeqValues;

JointTrajectoryDownloader::JointTrajectoryDownloader() : JointTrajectoryInterface()
{ 
}

bool JointTrajectoryDownloader::send_to_robot(const std::vector<JointTrajPtMessage>& messages)
{
  bool rslt=true;
  std::vector<JointTrajPtMessage> points(messages);
  SimpleMessage msg;

  // Trajectory download requires at least two points (START/END)
  if (points.size() < 2)
    points.push_back(JointTrajPtMessage(points[0]));

  // The first and last points are assigned special sequence values
  points.begin()->setSequence(SpecialSeqValues::START_TRAJECTORY_DOWNLOAD);
  points.back().setSequence(SpecialSeqValues::END_TRAJECTORY);

  if (!this->connection_->isConnected())
  {
    RCLCPP_WARN(this->get_logger(), "Attempting robot reconnection");
    this->connection_->makeConnect();
  }

  RCLCPP_INFO(this->get_logger(), "Sending trajectory points, size: %d", (int)points.size());

  for (int i = 0; i < (int)points.size(); ++i)
  {
    RCLCPP_DEBUG(this->get_logger(), "Sending joints trajectory point[%d]", i);

    points[i].toTopic(msg);
    bool ptRslt = this->connection_->sendMsg(msg);
    if (ptRslt)
      RCLCPP_DEBUG(this->get_logger(), "Point[%d] sent to controller", i);
    else
      RCLCPP_WARN(this->get_logger(), "Failed sent joint point, skipping point");
    rslt &= ptRslt;
  }

  return rslt;
}

} //joint_trajectory_downloader
} //industrial_robot_client

