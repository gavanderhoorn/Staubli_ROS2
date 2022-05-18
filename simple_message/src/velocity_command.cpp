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

#include "simple_message/velocity_command.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstring>

using namespace industrial::byte_array;
using namespace industrial::shared_types;

namespace industrial
{
namespace velocity_command
{

    VelocityCommand::VelocityCommand()
    {
        this->init();
    }

    VelocityCommand::~VelocityCommand()
    {
    }

    void VelocityCommand::init()
    {
        this->sequence_ = 0;
        this->type_ = 0;
        std::memset(this->vector_, 0, sizeof(this->vector_));
    }

    bool VelocityCommand::load(ByteArray *buffer)
    {
        //RCLCPP_INFO(rclcpp::get_logger("velocity_command"), "Executing velocity command load");

        if (!buffer->load(this->sequence_))
        {
            //RCLCPP_ERROR(rclcpp::get_logger("velocity_command"), "Failed to load velocity command sequence");
            return false;
        }

        for (const shared_real &value : this->vector_)
        {
            if (!buffer->load(value))
            {
                //RCLCPP_ERROR(rclcpp::get_logger("velocity_command"), "Failed to load velocity command vector");
                return false;
            }
        }

        if (!buffer->load(this->type_))
        {
            //RCLCPP_ERROR(rclcpp::get_logger("velocity_command"), "Failed to load velocity command type");
            return false;
        }

        return true;
    }

    bool VelocityCommand::unload(ByteArray *buffer)
    {
        //RCLCPP_INFO(rclcpp::get_logger("velocity_command"), "Executing velocity command unload");

        if (!buffer->unload(this->type_))
        {
            //RCLCPP_ERROR(rclcpp::get_logger("velocity_command"), "Failed to unload velocity command type");
            return false;
        }

        for (int i = MAX_NUM_JOINTS - 1; i >= 0; i--)
        {
            if (!buffer->unload(this->vector_[i]))
            {
                //RCLCPP_ERROR(rclcpp::get_logger("velocity_command"), "Failed to unload velocity command vector");
                return false;
            }
        }

        if (!buffer->unload(this->sequence_))
        {
            //RCLCPP_ERROR(rclcpp::get_logger("velocity_command"), "Failed to unload velocity command sequence");
            return false;
        }

        return true;
    }

} // namespace industrial
} // namespace velocity_command
