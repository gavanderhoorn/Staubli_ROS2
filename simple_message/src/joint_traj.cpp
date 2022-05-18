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
#include "simple_message/joint_traj.hpp"
#include "simple_message/shared_types.hpp"
#else
#include "joint_traj.hpp"
#include "shared_types.hpp"
#endif
#include "rclcpp/rclcpp.hpp"

using namespace industrial::shared_types;
using namespace industrial::joint_traj_pt;

namespace industrial
{
namespace joint_traj
{

JointTraj::JointTraj(void)
{
	this->init();
}
JointTraj::~JointTraj(void)
{

}

void JointTraj::init()
{
	JointTrajPt empty;

	this->size_ = 0;
	for (shared_int i = 0; i < this->getMaxNumPoints(); i++)
	{
		this->points_[i].copyFrom(empty);
	}
}

bool JointTraj::addPoint(JointTrajPt & point)
{
	bool rtn = false;

	if (!this->isFull())
	{
		this->points_[this->size()].copyFrom(point);
		this->size_++;
		rtn = true;
	}
	else
	{
		rtn = false;
		//RCLCPP_ERROR(rclcpp::get_logger("joint_traj"), "Failed to add point, buffer is full");
	}

	return rtn;
}

bool JointTraj::getPoint(shared_int index, JointTrajPt & point)
{
	bool rtn = false;

	if (index < this->size())
	{
		point.copyFrom(this->points_[index]);
		rtn = true;
	}
	else
	{
		//RCLCPP_ERROR(rclcpp::get_logger("joint_traj"), "Point index: %d, is greater than size: %d", index, this->size());
		rtn = false;
	}
	return rtn;
}

void JointTraj::copyFrom(JointTraj &src)
{
	JointTrajPt value;

	this->size_ = src.size();
	for (shared_int i = 0; i < this->size(); i++)
	{
		src.getPoint(i, value);
		this->points_[i].copyFrom(value);
	}
}

bool JointTraj::operator==(JointTraj &rhs)
{
	bool rtn = true;

	if(this->size() == rhs.size())
	{
		for(shared_int i = 0; i < this->size(); i++)
		{
			JointTrajPt value;
			rhs.getPoint(i, value);
			if(!(this->points_[i] == value))
			{
				//RCLCPP_DEBUG(rclcpp::get_logger("joint_traj"), "Joint trajectory point different");
				rtn = false;
				break;
			}
			else
			{
				rtn = true;
			}
		}
	}
	else
	{
		//RCLCPP_DEBUG(rclcpp::get_logger("joint_traj"), "Joint trajectory compare failed, size mismatch");
		rtn = false;
	}

	return rtn;
}


bool JointTraj::load(industrial::byte_array::ByteArray *buffer)
{
	bool rtn = false;
	JointTrajPt value;

	//RCLCPP_INFO(rclcpp::get_logger("joint_traj"), "Executing joint trajectory load");
	for (shared_int i = 0; i < this->size(); i++)
	{
		this->getPoint(i, value);
		rtn = buffer->load(value);
		if (!rtn)
		{
			//RCLCPP_ERROR(rclcpp::get_logger("joint_traj"), "Failed to load joint traj.pt. data");
			rtn = false;
			break;
		}
		rtn = true;
	}

	if (rtn)
	{
		rtn = buffer->load(this->size());
	}
	return rtn;
}

bool JointTraj::unload(industrial::byte_array::ByteArray *buffer)
{
	bool rtn = false;
	JointTrajPt value;

	//RCLCPP_INFO(rclcpp::get_logger("joint_traj"), "Executing joint trajectory unload");

	rtn = buffer->unload(this->size_);

	if(rtn)
	{
		for (int i = this->size() - 1; i >= 0; i--)
		{
			rtn = buffer->unload(value);
			if (!rtn)
			{
				//RCLCPP_ERROR(rclcpp::get_logger("joint_traj"), "Failed to unload message point: %d from data[%d]", i, buffer->getBufferSize());
				break;
			}
			this->points_[i].copyFrom(value);
		}
	}
	else
	{
		//RCLCPP_ERROR(rclcpp::get_logger("joint_traj"), "Failed to unload trajectory size");
	}
	return rtn;
}

}
}

