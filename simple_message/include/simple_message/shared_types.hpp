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

namespace industrial
{

/**
 * \brief Contains platform specific type definitions that guarantee the size
 * of primitive data types.
 *
 * The byte size of shared data types was determined by balancing the needs to
 * represent large data values with that of the platforms limits.  In the case
 * of platform limits, robot controllers represent the most restrictive limits.
 *
 * The majority of robot controllers have 32-bit architectures.  As such this was
 * chosen as the base for communications.  Real and Integer data types are represented
 * as 4 bytes.
 *
 * All types must match size and structure of the ROS_TYPES
 */
namespace shared_types
{

#if defined(INT32)
#include "stdint.h"
typedef int32_t shared_int;
#elif defined(INT64)
#include "stdint.h"
typedef int64_t shared_int;
#else
typedef int shared_int;
#endif

#ifndef FLOAT64
typedef float shared_real;
#else
typedef double shared_real;
#endif

typedef bool shared_bool;


} // namespace shared_types
} // namespace industrial

