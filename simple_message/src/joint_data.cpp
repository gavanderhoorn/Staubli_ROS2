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
#include "simple_message/joint_data.hpp"
#include "simple_message/shared_types.hpp"
#else
#include "joint_data.hpp"
#include "shared_types.hpp"
#endif
#include "rclcpp/rclcpp.hpp"

using namespace industrial::shared_types;

namespace industrial
{
namespace joint_data
{

JointData::JointData(void)
{
  this->init();
}
JointData::~JointData(void)
{

}

void JointData::init()
{
  for (int i = 0; i < this->getMaxNumJoints(); i++)
  {
    this->setJoint(i, 0.0);
  }
}

bool JointData::setJoint(shared_int index, shared_real value)
{
  bool rtn = false;

  if (index < this->getMaxNumJoints())
  {
    this->joints_[index] = value;
    rtn = true;
  }
  else
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_data"), "Joint index: %d, is greater than size: %d", index, this->getMaxNumJoints());
    rtn = false;
  }
  return rtn;
}

bool JointData::getJoint(shared_int index, shared_real & value) const
{
  bool rtn = false;

  if (index < this->getMaxNumJoints())
  {
    value = this->joints_[index];
    rtn = true;
  }
  else
  {
    //RCLCPP_ERROR(rclcpp::get_logger("joint_data"), "Joint index: %d, is greater than size: %d", index, this->getMaxNumJoints());
    rtn = false;
  }
  return rtn;
}

shared_real JointData::getJoint(shared_int index) const
{
  shared_real rtn = 0.0;
  this->getJoint(index, rtn);
  return rtn;
}
  

void JointData::copyFrom(JointData &src)
{
  shared_real value = 0.0;

  for (int i = 0; i < this->getMaxNumJoints(); i++)
  {
    src.getJoint(i, value);
    this->setJoint(i, value);
  }
}

bool JointData::operator==(JointData &rhs)
{
  bool rtn = true;

  shared_real lhsvalue, rhsvalue;

  for (int i = 0; i < this->getMaxNumJoints(); i++)
  {
    this->getJoint(i, lhsvalue);
    rhs.getJoint(i, rhsvalue);
    if (lhsvalue != rhsvalue)
    {
      rtn = false;
      break;
    }
  }
  return rtn;

}

bool JointData::load(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;
  shared_real value = 0.0;

  //RCLCPP_INFO(rclcpp::get_logger("joint_data"), "Executing joint position load");
  for (int i = 0; i < this->getMaxNumJoints(); i++)
  {
    this->getJoint(i, value);
    rtn = buffer->load(value);
    if (!rtn)
    {
      //RCLCPP_ERROR(rclcpp::get_logger("joint_data"), "Failed to load joint position data");
      break;
    }
  }
  return rtn;
}

bool JointData::unload(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;
  shared_real value = 0.0;

  //RCLCPP_INFO(rclcpp::get_logger("joint_data"), "Executing joint position unload");
  for (int i = this->getMaxNumJoints() - 1; i >= 0; i--)
  {
    rtn = buffer->unload(value);
    if (!rtn)
    {
      //RCLCPP_ERROR(rclcpp::get_logger("joint_data"), "Failed to unload message joint: %d from data[%d]", i, buffer->getBufferSize());
      break;
    }
    this->setJoint(i, value);
  }
  return rtn;
}

}
}

