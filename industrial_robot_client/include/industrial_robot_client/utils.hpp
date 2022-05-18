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
#include <map>

namespace industrial_robot_client
{
namespace utils
{


/**
 * \brief Checks to see if members are within the same range.  Specifically,
 * Each item in abs(lhs[i] - rhs[i]) < abs(range/2.0).  Empty vectors always
 * return true.
 * The behavior of this function around large values (near double limits) is
 * uncertain.
 * This function is not optimized, do not use for large vectors or loops.
 *
 * \param lhs first vector
 * \param rhs second vector
 * \param full_range
 *
 * \return true set are similar (same members, same order)
 */
bool isWithinRange(const std::vector<double> & lhs, const std::vector<double> & rhs,
                   double full_range);

/**
 * \brief Convenience function for inserting a value into an std::map, assuming
 * a string key and a double value
 *
 * \param key
 * \param value
 * \param map
 *
 * \return true if insertion successful (or if key already exists)
 */
bool mapInsert(const std::string & key, double value, std::map<std::string, double> & mappings);


/**
 * \brief Convenience function for taking two separate key,value vectors and creating
 * the more convenient map.  This is helpful for some ROS message types where maps are
 * defined as two separate vectors (ROS messages do not support maps).
 *
 * \param keys vector of key values
 * \param values vector of values
 * \param mappings of key values.
 *
 * \return true if conversion successful
 */
bool toMap(const std::vector<std::string> & keys, const std::vector<double> & values,
           std::map<std::string, double> & mappings);


/**
 * \brief Map version of @see isWithinRange
 *
 * \param keys vector of key values
 * \param lhs first map
 * \param rhs second map
 * \param full_range
 *
 * \return true if values are within range
 */
bool isWithinRange(const std::vector<std::string> & keys, const std::map<std::string, double> & lhs,
                   const std::map<std::string, double> & rhs, double full_range);


/**
 * \brief Key/Values vector version of @see isWithinRange
 *
 * \param lhs_keys vector of lhs keys
 * \param lhs_values vector of lhs_values
 * \param rhs_keys vector of rhs keys
 * \param rhs_values vector of rhs_values
 * \param full_range
 *
 * \return true values within range
 */
bool isWithinRange(const std::vector<std::string> & lhs_keys, const std::vector<double> & lhs_values,
                   const std::vector<std::string> & rhs_keys, const std::vector<double> & rhs_values,
                   double full_range);

} //utils
} //industrial_robot_client

