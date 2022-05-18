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
#include "simple_message/joint_data.hpp"
#include "simple_message/simple_message.hpp"
#include "simple_message/simple_serialize.hpp"
#include "simple_message/shared_types.hpp"
#else
#include "joint_data.hpp"
#include "simple_message.hpp"
#include "simple_serialize.hpp"
#include "shared_types.hpp"
#endif

namespace industrial
{
namespace joint_traj_pt
{

namespace SpecialSeqValues
{
enum SpecialSeqValue
{
  START_TRAJECTORY_DOWNLOAD  = -1, ///< Downloading drivers only: signal start of trajectory
  START_TRAJECOTRY_STREAMING = -2, ///< deprecated, please use START_TRAJECTORY_STREAMING instead
  START_TRAJECTORY_STREAMING = -2, ///< Streaming drivers only: signal start of trajectory
  END_TRAJECTORY  = -3, ///< Downloading drivers only: signal end of trajectory
  STOP_TRAJECTORY = -4  ///< Server should stop the current motion (if any) as soon as possible
};
}
typedef SpecialSeqValues::SpecialSeqValue SpecialSeqValue;

/**
 * \brief Class encapsulated joint trajectory point data.  The point data
 * serves as a waypoint along a trajectory and is meant to mirror the
 * JointTrajectoryPoint message.
 *
 * This point differs from the ROS trajectory point in the following ways:
 *
 *  - The joint velocity in an industrial robot standard way (as a single value).
 *  - The duration is somewhat different than the ROS timestamp.  The timestamp
 *    specifies when the move should start, where as the duration is how long the
 *    move should take.  A big assumption is that a sequence of points is continuously
 *    executed.  This is generally true of a ROS trajectory but not required.
 *
 * The byte representation of a joint trajectory point is as follow (in order lowest index
 * to highest). The standard sizes are given, but can change based on type sizes:
 *
 *   member:             type                                      size
 *   sequence            (industrial::shared_types::shared_int)    4  bytes
 *   joints              (industrial::joint_data)                  40 bytes
 *   velocity            (industrial::shared_types::shared_real)   4  bytes
 *   duration            (industrial::shared_types::shared_real)   4  bytes
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class JointTrajPt : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  JointTrajPt(void);
  /**
   * \brief Destructor
   *
   */
  ~JointTrajPt(void);

  /**
   * \brief Initializes a empty joint trajectory point
   *
   */
  void init();

  /**
   * \brief Initializes a complete trajectory point
   *
   */
  void init(industrial::shared_types::shared_int sequence, industrial::joint_data::JointData & position,
            industrial::shared_types::shared_real velocity, industrial::shared_types::shared_real duration);

  /**
   * \brief Sets joint position data
   *
   * \param joint position data
   */
  void setJointPosition(industrial::joint_data::JointData &position)
  {
    this->joint_position_.copyFrom(position);
  }

  /**
   * \brief Returns a copy of the position data
   *
   * \param joint position dest
   */
  void getJointPosition(industrial::joint_data::JointData &dest)
  {
    dest.copyFrom(this->joint_position_);
  }

  /**
   * \brief Sets joint trajectory point sequence number
   *
   * \param sequence value
   */
  void setSequence(industrial::shared_types::shared_int sequence)
  {
    this->sequence_ = sequence;
  }

  /**
   * \brief Returns joint trajectory point sequence number
   *
   * \return joint trajectory sequence number
   */
  industrial::shared_types::shared_int getSequence()
  {
    return this->sequence_;
  }

  /**
   * \brief Sets joint trajectory point velocity
   *
   * \param velocity value
   */
  void setVelocity(industrial::shared_types::shared_real velocity)
  {
    this->velocity_ = velocity;
  }

  /**
   * \brief Returns joint trajectory point velocity
   *
   * \return joint trajectory velocity
   */
  industrial::shared_types::shared_real getVelocity()
  {
    return this->velocity_;
  }

  /**
     * \brief Sets joint trajectory point duration
     *
     * \param velocity value
     */
    void setDuration(industrial::shared_types::shared_real duration)
    {
      this->duration_ = duration;
    }

    /**
     * \brief Returns joint trajectory point duration
     *
     * \return joint trajectory velocity
     */
    industrial::shared_types::shared_real getDuration()
    {
      return this->duration_;
    }

  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(JointTrajPt &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(JointTrajPt &rhs);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return sizeof(industrial::shared_types::shared_real) + sizeof(industrial::shared_types::shared_int)
        + this->joint_position_.byteLength();
  }

private:

  /**
   * \brief joint point positional data
   */
  industrial::joint_data::JointData joint_position_;
  /**
   * \brief joint point velocity
   */
  industrial::shared_types::shared_real velocity_;
  /**
   * \brief trajectory sequence number
   */
  industrial::shared_types::shared_int sequence_;

  /**
   * \brief joint move duration
   */
  industrial::shared_types::shared_real duration_;

};

}
}