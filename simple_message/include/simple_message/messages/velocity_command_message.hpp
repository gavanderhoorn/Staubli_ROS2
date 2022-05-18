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

#include "simple_message/velocity_command.hpp"

#include "simple_message/byte_array.hpp"
#include "simple_message/simple_message.hpp"
#include "simple_message/typed_message.hpp"

namespace industrial
{
namespace velocity_command_message
{

class VelocityCommandMessage : public industrial::typed_message::TypedMessage
{
public:
  VelocityCommandMessage(void);
  virtual ~VelocityCommandMessage(void);

  void init(void) override;
  void init(const industrial::velocity_command::VelocityCommand& data);
  bool init(industrial::simple_message::SimpleMessage& msg) override;

  unsigned int byteLength() override
  {
    return this->data_.byteLength();
  }

  bool load(industrial::byte_array::ByteArray* buffer) override;
  bool unload(industrial::byte_array::ByteArray* buffer) override;

  industrial::velocity_command::VelocityCommand data_;
};

}  // namespace industrial
}  // namespace velocity_command_message
