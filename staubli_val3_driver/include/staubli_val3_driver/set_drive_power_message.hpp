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

#include "simple_message/typed_message.hpp"
#include "simple_message/simple_message.hpp"
#include "simple_message/shared_types.hpp"

/**
 * @brief Message representing the industrial_msgs/SetDrivePower service
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class SetDrivePowerMessage : public industrial::typed_message::TypedMessage
{
public:
  /**
   * @brief Default constructor
   *
   * This method creates an empty message.
   *
   */
  SetDrivePowerMessage();
  /**
   * @brief Destructor
   *
   */
  ~SetDrivePowerMessage();
  /**
   * @brief Initializes message from a simple message
   *
   * @param simple message to construct from
   *
   * @return true if message successfully initialized, otherwise false
   */
  bool init(industrial::simple_message::SimpleMessage &msg);

  /**
   * @brief Initializes message
   *
   * @param drive power value to initialize from
   *
   */
  void init(industrial::shared_types::shared_bool drive_power);

  /**
   * @brief Initializes a new set drive power message
   *
   */
  void init();

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);

  unsigned int byteLength()
  {
    return sizeof(industrial::shared_types::shared_bool);
  }

  industrial::shared_types::shared_bool drive_power_;
};