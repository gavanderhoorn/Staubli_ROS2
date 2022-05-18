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


#include "urdf_extention/model_extention.hpp"
using namespace std::chrono_literals;

namespace urdf
{
ModelExtention::ModelExtention() : Model(), Node("model_extention")
{
}
ModelExtention::~ModelExtention()
{
}
bool ModelExtention::initParam(const std::string &node_name, const std::string &param_name)
{
    std::vector <rclcpp::Parameter> xml_strings = {};
    rclcpp::Parameter xml_string;

    rclcpp::SyncParametersClient::SharedPtr parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, node_name);
    while (!parameters_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

    if(!parameters_client->has_parameter(param_name))
    {
        RCLCPP_ERROR(this->get_logger(), "Parameter %s not found in node %s", param_name.c_str(), node_name.c_str());
        return false;
    }

    xml_strings = parameters_client->get_parameters({param_name});
    xml_string = xml_strings[0];

    if(!xml_string.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
    {
        RCLCPP_ERROR(this->get_logger(), "Parameter " + param_name + " of node " + node_name + " is not of type string");
    }
    
    // RCLCPP_INFO(this->get_logger(), xml_string.as_string());

    return this->initString(xml_string.as_string());
}
}
