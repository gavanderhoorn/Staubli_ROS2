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

#include "simple_message/simple_message.hpp"
#include "simple_message/simple_serialize.hpp"
#include "simple_message/shared_types.hpp"

/**
 * @brief Enumeration mirrors staubli_msgs/IOModule definition
 * 
 */
enum IOModule
{
    UNKNOWN = -1,
    USER_IN = 1,
    VALVE_OUT = 2,
    BASIC_IN = 3,
    BASIC_OUT = 4,
    BASIC_IN_2 = 5,
    BASIC_OUT_2 = 6
};

class WriteSingleIO : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * @brief Default constructor
   *
   * This method creates empty data.
   *
   */
  WriteSingleIO();

  /**
   * @brief Destructor
   *
   */
  ~WriteSingleIO();

  /**
   * @brief Initializes an empty WriteSingleIO structure
   *
   */
  void init();

  /**
   * @brief Initializes a full WriteSingleIO structure
   *
   */
  void init(IOModule moduleId, industrial::shared_types::shared_int pin, industrial::shared_types::shared_bool state);

  IOModule getModuleId()
  {
    return IOModule(module_id_);
  }

  industrial::shared_types::shared_int getPin() const
  {
    return pin_;
  }

  industrial::shared_types::shared_bool getState() const
  {
    return state_;
  }

  void setModuleId(industrial::shared_types::shared_int module_id)
  {
    this->module_id_ = module_id;
  }

  void setPin(industrial::shared_types::shared_int pin)
  {
    this->pin_ = pin;
  }

  void setState(industrial::shared_types::shared_bool state)
  {
    this->state_ = state;
  }

  /**
   * @brief Copies the passed in value
   *
   * @param src (value to copy)
   */
  void copyFrom(WriteSingleIO& src);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray* buffer);
  bool unload(industrial::byte_array::ByteArray* buffer);
  unsigned int byteLength()
  {
    return 2 * sizeof(industrial::shared_types::shared_int) + sizeof(industrial::shared_types::shared_bool);
  }

private:
  /**
   * @brief Identifier of the IOModule (see staubli_msgs/IOModule)
   */
  industrial::shared_types::shared_int module_id_;

  /**
   * @brief Index of the pin
   */
  industrial::shared_types::shared_int pin_;

  /**
   * @brief Pin state
   */
  industrial::shared_types::shared_bool state_;
};