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

#ifndef FLATHEADERS
#include "simple_message/byte_array.hpp"
#else
#include "byte_array.hpp"
#endif

#include "string.h"

namespace industrial
{
namespace simple_serialize
{


  /**
   * \brief Interface for loading and unloading a class to/from a ByteArray
   */
class SimpleSerialize
{
public:
  /**
   * \brief Virtual method for loading an object into a ByteArray
   *
   * This method should load all the required data to reconstruct the class
   * object into the buffer
   *
   * \param buffer pointer to ByteArray
   *
   * \return true on success, false otherwise (buffer not large enough)
   */
  virtual bool load(industrial::byte_array::ByteArray *buffer)=0;

  /**
   * \brief Virtual method for unloading an object from a ByteArray
   *
   * This method should unload all the required data to reconstruct
   * the class object (in place)
   *
   * \param buffer pointer to ByteArray
   *
   * \return true on success, false otherwise (buffer not large enough)
   */
  virtual bool unload(industrial::byte_array::ByteArray *buffer)=0;

  /**
   * \brief Virtual method returns the object size when packed into a
   * ByteArray
   *
   * \return object size (in bytes)
   */
  virtual unsigned int byteLength()=0;

};

} // namespace simple_serialize
} // namespace industrial
