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
#include "urdf_extention/model_extention.hpp"

namespace industrial_utils
{

/**
 * \brief Checks to see if sets are similar(same members, any order)
 * NOTE: Vectors are passed by copy because they are modified by the
 * function(i.e. ordered).
 * This function should not be used for large vectors or in loops (it
 * is slow!)
 *
 * \param lhs first vector
 * \param rhs second vector
 *
 * \return true set are similar (same members, any order)
 */
bool isSimilar(std::vector<std::string> lhs, std::vector<std::string> rhs);

/**
 * \brief Checks to see if sets are the same(same members, same order)
 * Wraps std::equal method.
 *
 * \param lhs first vector
 * \param rhs second vector
 *
 * \return true set are similar (same members, same order)
 */
bool isSame(const std::vector<std::string> & lhs, const std::vector<std::string> & rhs);

/*
 * \brief Returns joint names for a simple serial chain from a URDF tree
 *          - returns an error if branching tree is found
 *          - assumes chain runs root->leaf.  leaf->root->leaf chains not allowed.
 *
 * \param[in] link URDF model link to search from (e.g. model.getRoot())
 * \param[in] ignore_fixed flag to ignore fixed joints
 * \param[in,out] joint_names vector of joint names
 *
 * \return true if successful, false if error occurred (e.g. branching tree)
 */
bool findChainJointNames(const urdf::LinkConstSharedPtr &link, bool ignore_fixed,
		                 std::vector<std::string> &joint_names);

} //industrial_utils
