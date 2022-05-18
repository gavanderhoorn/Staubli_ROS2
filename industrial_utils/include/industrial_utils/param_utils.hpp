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

#include <map>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"

namespace industrial_utils
{
namespace param
{

class ParamUtils : public rclcpp::Node
{
public:
	ParamUtils();

	/**
	 * \brief Gets parameter list as vector of strings
	 *
	 * \param param_name name of list parameter
	 * \param list_param populated with parameter value(s)
	 *
	 * \return true if parameter
	 */
	bool getListParam(const std::string node_name, const std::string param_name, std::vector<std::string> & list_param);

	std::string vec2str(const std::vector<std::string> &vec);

	/**
	 * \brief Tries to get a set of joint names using several fallback methods:
	 *          1) check parameter for an explicit list
	 *          2) try to parse from given URDF data
	 *          3) use default joint names: ["joint_1", "joint_2", ..., "joint_6"]
	 *
	 * \param[in] joint_list_param name of joint-names-list parameter to check
	 * \param[in] urdf_param name of URDF description parameter to check
	 * \param[out] joint_names list of joint names
	 *
	 * \return true if parameter found, false if defaults used
	 */
	bool getJointNames(const std::string joint_names_node_name, const std::string urdf_node_name, const std::string joint_list_param, const std::string urdf_param,
					std::vector<std::string> & joint_names);

	/**
	 * \brief Tries to read joint velocity limits from the specified URDF parameter
	 *
	 * \param[in] urdf_param_name name of URDF parameter
	 * \param[out] velocity_limits map of velocity limits for each URDF joint
	 *
	 * \return true if parameter found, false if not found
	 */
	bool getJointVelocityLimits(const std::string node_name, const std::string urdf_param_name, std::map<std::string, double> &velocity_limits);

};																																																																											
} //industrial_utils::param
} //industrial_utils
