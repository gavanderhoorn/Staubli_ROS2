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

#include "staubli_val3_driver/write_single_io.hpp"
#include "rclcpp/rclcpp.hpp"

WriteSingleIO::WriteSingleIO()
{
  this->init();
}

WriteSingleIO::~WriteSingleIO()
{
}

void WriteSingleIO::init()
{
  this->init(IOModule::UNKNOWN, 0, false);
}

void WriteSingleIO::init(IOModule module_id, 
                         industrial::shared_types::shared_int pin, 
                         industrial::shared_types::shared_bool state)
{
  this->setModuleId(module_id);
  this->setPin(pin);
  this->setState(state);
}

void WriteSingleIO::copyFrom(WriteSingleIO& src)
{
  this->init(src.getModuleId(), src.getPin(), src.getState());
}

bool WriteSingleIO::load(industrial::byte_array::ByteArray* buffer)
{
  bool rtn = false;

  RCLCPP_INFO(rclcpp::get_logger("write_single_io"),"Executing WriteSingleIO load");

  if (buffer->load(this->module_id_) && buffer->load(this->pin_) && buffer->load(this->state_))
  {

    RCLCPP_INFO(rclcpp::get_logger("write_single_io"), "WriteSingleIO successfully loaded");
    rtn = true;
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("write_single_io"), "WriteSingleIO not loaded");
    rtn = false;
  }

  return rtn;
}

bool WriteSingleIO::unload(industrial::byte_array::ByteArray* buffer)
{
  bool rtn = false;

  RCLCPP_INFO(rclcpp::get_logger("write_single_io"), "Executing WriteSingleIO unload");
  if (buffer->unload(this->state_) && buffer->unload(this->pin_) && buffer->unload(this->module_id_))
  {
    rtn = true;
    RCLCPP_INFO(rclcpp::get_logger("write_single_io"), "WriteSingleIO successfully unloaded");
  }

  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("write_single_io"), "Failed to unload WriteSingleIO");
    rtn = false;
  }

  return rtn;
}