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

class IOStates : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  IOStates();

  /**
   * \brief Destructor
   *
   */
  ~IOStates();

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray* buffer);
  bool unload(industrial::byte_array::ByteArray* buffer);
  unsigned int byteLength()
  {
    return 6 * sizeof(industrial::shared_types::shared_int) + 2 * sizeof(industrial::shared_types::shared_bool);
  }

private:
  /**
   * \brief UserIO inputs
   */
  industrial::shared_types::shared_int user_in_;

  /**
   * \brief UserIO outputs (valve)
   */
  industrial::shared_types::shared_int valve_out_;

  /**
   * \brief BasicIO inputs
   */
  industrial::shared_types::shared_int basic_in_;

  /**
   * \brief BasicIO outputs
   */
  industrial::shared_types::shared_int basic_out_;

  /**
   * \brief BasicIO-2 inputs (2nd IO module)
   */
  industrial::shared_types::shared_int basic_in_2_;

  /**
   * \brief BasicIO-2 outputs (2nd IO module)
   */
  industrial::shared_types::shared_int basic_out_2_;

  /**
   * \brief Flag indicating if BasicIO module 1 is working
   */
  industrial::shared_types::shared_bool basic_io_valid_;

  /**
   * \brief Flag indicating if BasicIO module 2 is working
   */
  industrial::shared_types::shared_bool basic_io_2_valid_;
};