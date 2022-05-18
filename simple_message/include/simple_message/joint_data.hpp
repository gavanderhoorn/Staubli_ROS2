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
#include "simple_message/simple_message.hpp"
#include "simple_message/simple_serialize.hpp"
#include "simple_message/shared_types.hpp"
#else
#include "simple_message.hpp"
#include "simple_serialize.hpp"
#include "shared_types.hpp"
#endif

namespace industrial
{
namespace joint_data
{

/**
 * \brief Class encapsulated joint data (positions, accelerations, velocity,
 * torque, and/or effort).
 *
 * For simplicity and cross platform compliance, this is implemented as a
 * fixed size array.
 *
 * The byte representation of a joint data is as follows. The standard sizes
 * are given, but can change based on type sizes:
 *
 *   member:             type                                      size
 *   joints              (industrial::shared_types::shared_real)   4 * MAX_NUM_JOINTS
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class JointData : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  JointData(void);
  /**
   * \brief Destructor
   *
   */
  ~JointData(void);

  /**
   * \brief Initializes a empty joint data
   *
   */
  void init();

  /**
   * \brief Sets a joint value within the buffer
   *
   * \param joint index
   * \param joint value
   *
   * \return true if value set, otherwise false (index greater than max)
   */
  bool setJoint(industrial::shared_types::shared_int index, industrial::shared_types::shared_real value);

  /**
   * \brief Gets a joint value within the buffer
   *
   * \param joint index
   * \param joint value (passed by reference)
   *
   * \return true if value valid, otherwise false (index greater than max)
   */
  bool getJoint(industrial::shared_types::shared_int index, industrial::shared_types::shared_real & value) const;

  /**
   * \brief Gets a joint value within the buffer (Only use this form if you are
   * sure the index is within bounds).
   *
   * \param joint index
   *
   * \return joint value (returns 0.0 if index is out of bounds)
   */
  industrial::shared_types::shared_real getJoint(industrial::shared_types::shared_int index) const;

  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(JointData &src);

  /**
   * \brief returns the maximum number of joints the message holds
   *
   * \return max number of joints
   */
  int getMaxNumJoints() const
  {
    return MAX_NUM_JOINTS;
  }

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(JointData &rhs);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return MAX_NUM_JOINTS * sizeof(industrial::shared_types::shared_real);
  }

private:

  /**
   * \brief maximum number of joints positions that can be held in the message.
   */
  static const industrial::shared_types::shared_int MAX_NUM_JOINTS = 10;
  /**
   * \brief internal data buffer
   */
  industrial::shared_types::shared_real joints_[MAX_NUM_JOINTS];

};

}
}