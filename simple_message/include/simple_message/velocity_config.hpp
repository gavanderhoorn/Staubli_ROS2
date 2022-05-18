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

#include "simple_message/byte_array.hpp"
#include "simple_message/shared_types.hpp"
#include "simple_message/simple_serialize.hpp"

namespace industrial
{
namespace velocity_config
{

class VelocityConfig : public industrial::simple_serialize::SimpleSerialize
{
public:
  VelocityConfig();

  ~VelocityConfig();

  void init(void);

  /**
   * Overrides - SimpleSerialize
   */
  unsigned int byteLength()
  {
    return sizeof(industrial::shared_types::shared_int) + (4 + 2 * 6) * sizeof(industrial::shared_types::shared_real);
  }

  bool load(industrial::byte_array::ByteArray* buffer);
  bool unload(industrial::byte_array::ByteArray* buffer);

  industrial::shared_types::shared_int cmd_type_;
  industrial::shared_types::shared_real frame_ref_[6];
  industrial::shared_types::shared_real tool_ref_[6];
  industrial::shared_types::shared_real accel_;  // maximum joint acceleration in % of the nominal acceleration [0,1]
  industrial::shared_types::shared_real vel_;    // maximum joint velocity in % of the nominal velocity [0,1]
  industrial::shared_types::shared_real tvel_;   // maximum linear velocity of the tool-center-point in m/s
  industrial::shared_types::shared_real rvel_;   // maximum angular velocity of the tool-center-point in rad/s
};

}  // namespace industrial
}  // namespace velocity_config
