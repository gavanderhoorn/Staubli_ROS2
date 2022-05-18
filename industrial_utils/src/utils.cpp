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

#include "industrial_utils/utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>

namespace industrial_utils
{

bool isSimilar(std::vector<std::string> lhs, std::vector<std::string> rhs)
{
  bool rtn = false;

  if (lhs.size() != rhs.size())
  {
    rtn = false;
  }
  else
  {
    std::sort(lhs.begin(), lhs.end());
    std::sort(rhs.begin(), rhs.end());
    rtn = isSame(lhs, rhs);
  }

  return rtn;
}

bool isSame(const std::vector<std::string> & lhs, const std::vector<std::string> & rhs)
{
  bool rtn = false;

  if (lhs.size() != rhs.size())
  {
    rtn = false;
  }
  else
  {
    rtn = std::equal(lhs.begin(), lhs.end(), rhs.begin());
  }
  return rtn;
}

bool findChainJointNames(const urdf::LinkConstSharedPtr &link, bool ignore_fixed,
		                 std::vector<std::string> &joint_names)
{
  typedef std::vector<urdf::JointSharedPtr > joint_list;
  typedef std::vector<urdf::LinkSharedPtr > link_list;
  std::string found_joint, found_link;

  // check for joints directly connected to this link
  const joint_list &joints = link->child_joints;
  RCLCPP_DEBUG(rclcpp::get_logger("utils"), "Found %lu child joints:", joints.size());
  for (joint_list::const_iterator it=joints.begin(); it!=joints.end(); ++it)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("utils"), "  " << (*it)->name << ": type " <<  (*it)->type);
    if (ignore_fixed && (*it)->type == urdf::Joint::FIXED)
      continue;

    if (found_joint.empty())
    {
      found_joint = (*it)->name;
      joint_names.push_back(found_joint);
    }
    else
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("utils"), "Unable to find chain in URDF.  Branching joints: " << found_joint << " and " << (*it)->name);
      return false;  // branching tree (multiple valid child-joints)
    }
  }

  // check for joints connected to children of this link
  const link_list &links = link->child_links;
  std::vector<std::string> sub_joints;
  RCLCPP_DEBUG(rclcpp::get_logger("utils"), "Found %lu child links:", links.size());
  for (link_list::const_iterator it=links.begin(); it!=links.end(); ++it)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("utils"), "  " << (*it)->name);
    if (!findChainJointNames(*it, ignore_fixed, sub_joints))   // NOTE: recursive call
      return false;

    if (sub_joints.empty())
      continue;

    if (found_link.empty())
    {
      found_link = (*it)->name;
      joint_names.insert(joint_names.end(), sub_joints.begin(), sub_joints.end());  // append sub_joints
    }
    else
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("utils"), "Unable to find chain in URDF.  Branching links: " << found_link << " and " << (*it)->name);
      return false;  // branching tree (multiple valid child-joints)
    }
  }

  return true;
}

} //industrial_utils
