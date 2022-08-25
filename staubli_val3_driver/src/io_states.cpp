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

#include "staubli_val3_driver/io_states.hpp"

#include "simple_message/shared_types.hpp"
#include "rclcpp/rclcpp.hpp"

IOStates::IOStates()
{
}

IOStates::~IOStates()
{
}

bool IOStates::load(industrial::byte_array::ByteArray* buffer)
{
  bool rtn = false;

  RCLCPP_INFO(rclcpp::get_logger("io_states"), "Executing IO states load");

  if (buffer->load(this->user_in_) && buffer->load(this->valve_out_) && buffer->load(this->basic_in_) &&
      buffer->load(this->basic_out_) && buffer->load(this->basic_in_2_) && buffer->load(this->basic_out_2_) &&
      buffer->load(this->basic_io_valid_) && buffer->load(this->basic_io_2_valid_))
  {

    RCLCPP_INFO(rclcpp::get_logger("io_states"), "IO states successfully loaded");
    rtn = true;
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("io_states"), "IO states not loaded");
    rtn = false;
  }

  return rtn;
}

bool IOStates::unload(industrial::byte_array::ByteArray* buffer)
{
  bool rtn = false;

  RCLCPP_INFO(rclcpp::get_logger("io_states"), "Executing IO states unload");
  if (buffer->unload(this->basic_io_2_valid_) && buffer->unload(this->basic_io_valid_) &&
      buffer->unload(this->basic_out_2_) && buffer->unload(this->basic_in_2_) && buffer->unload(this->basic_out_) &&
      buffer->unload(this->basic_in_) && buffer->unload(this->valve_out_) && buffer->unload(this->user_in_))
  {

    rtn = true;
    RCLCPP_INFO(rclcpp::get_logger("io_states"), "IO states successfully unloaded");
  }

  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("io_states"), "Failed to unload IO states");
    rtn = false;
  }

  return rtn;
}
