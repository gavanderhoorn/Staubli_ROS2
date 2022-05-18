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

#include "rclcpp/rclcpp.hpp"

#include "industrial_utils/utils.hpp"
#include "industrial_robot_client/utils.hpp"

#include <cmath>
#include <iostream>

namespace industrial_robot_client
{
namespace utils
{

bool isWithinRange(const std::vector<double> & lhs, const std::vector<double> & rhs, double full_range)
{
  bool rtn = false;

  if (lhs.size() != rhs.size())
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("utils"), __FUNCTION__ << "::lhs size: " << lhs.size() << " does not match rhs size: " << rhs.size());
    rtn = false;
  }
  else
  {
    // Calculating the half range causes some precision loss, but it's good enough
    double half_range = full_range / 2.0;
    rtn = true; // Assume within range, catch exception in loop below

    // This loop will not run for empty vectors, results in return of true
    for (size_t i = 0; i < lhs.size(); ++i)
    {
      if (fabs(lhs[i] - rhs[i]) > fabs(half_range))
      {
        rtn = false;
        break;
      }
    }

  }

  return rtn;
}

bool mapInsert(const std::string & key, double value, std::map<std::string, double> & mappings)
{
  bool rtn = false;

  std::pair<std::map<std::string, double>::iterator, bool> insert_rtn;

  insert_rtn = mappings.insert(std::make_pair(key, value));

  // The second value returned form insert is a boolean (true for success)
  if (!insert_rtn.second)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("utils"), __FUNCTION__ << "::Failed to insert item into map with key: " << key);
    rtn = false;
  }
  else
  {
    rtn = true;
  }
  return rtn;

}

bool toMap(const std::vector<std::string> & keys, const std::vector<double> & values,
           std::map<std::string, double> & mappings)
{
  bool rtn;

  mappings.clear();

  if (keys.size() == values.size())
  {
    rtn = true;

    for (size_t i = 0; i < keys.size(); ++i)
    {
      rtn = mapInsert(keys[i], values[i], mappings);
      if (!rtn)
      {
        break;
      }
    }

  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("utils"), __FUNCTION__ << "::keys size: " << keys.size()
                     << " does not match values size: " << values.size());

    rtn = false;
  }

  return rtn;
}

bool isWithinRange(const std::vector<std::string> & keys, const std::map<std::string, double> & lhs,
                   const std::map<std::string, double> & rhs, double full_range)
{
  bool rtn = false;

  if ((keys.size() != rhs.size()) || (keys.size() != lhs.size()))
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("utils"), __FUNCTION__ << "::Size mistmatch ::lhs size: " << lhs.size() <<
                     " rhs size: " << rhs.size() << " key size: " << keys.size());

    rtn = false;
  }
  else
  {
    // Calculating the half range causes some precision loss, but it's good enough
    double half_range = full_range / 2.0;
    rtn = true; // Assume within range, catch exception in loop below

    // This loop will not run for empty vectors, results in return of true
    for (size_t i = 0; i < keys.size(); ++i)
    {
      if (fabs(lhs.at(keys[i]) - rhs.at(keys[i])) > fabs(half_range))
      {
        rtn = false;
        break;
      }
    }

  }

  return rtn;
}

bool isWithinRange(const std::vector<std::string> & lhs_keys, const std::vector<double> & lhs_values,
                   const std::vector<std::string> & rhs_keys, const std::vector<double> & rhs_values, double full_range)
{
  bool rtn = false;
  std::map<std::string, double> lhs_map;
  std::map<std::string, double> rhs_map;
  if (industrial_utils::isSimilar(lhs_keys, rhs_keys))
  {
    if (toMap(lhs_keys, lhs_values, lhs_map) && toMap(rhs_keys, rhs_values, rhs_map))
    {
      rtn = isWithinRange(lhs_keys, lhs_map, rhs_map, full_range);
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("utils"), __FUNCTION__ << "::Key vectors are not similar");
    rtn = false;
  }
  return rtn;
}

} //utils
} //industrial_robot_client
