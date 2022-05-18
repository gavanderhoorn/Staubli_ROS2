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
namespace robot_status
{

/**
 * \brief Enumeration mirrors industrial_msgs/RobotMode definition
 *
 */
namespace RobotModes
{
enum RobotMode
{
  UNKNOWN = -1,

  MANUAL = 1, AUTO = 2,
};

// remove ROS after Melodic (bw compat for #262)
#if defined(SIMPLE_MESSAGE_USE_ROS) || defined(ROS)
int toROSMsgEnum(RobotModes::RobotMode mode);
#endif

}
typedef RobotModes::RobotMode RobotMode;

/**
 * \brief Enumeration mirrors industrial_msgs/TriState definition.
 * NOTE: The TS prefix is needed because the ON and TRUE value collide
 * with other defined types on some systems.
 *
 */
namespace TriStates
{

enum TriState
{
  TS_UNKNOWN = -1,
  // These values must all be the same
  TS_TRUE = 1,   TS_ON = 1,  TS_ENABLED = 1,  TS_HIGH = 1,
  // These values must all be the same
  TS_FALSE = 0,   TS_OFF = 0,  TS_DISABLED = 0,  TS_LOW = 0
};

// remove ROS after Melodic (bw compat for #262)
#if defined(SIMPLE_MESSAGE_USE_ROS) || defined(ROS)
int toROSMsgEnum(TriStates::TriState state);
#endif

}
typedef TriStates::TriState TriState;

/**
 * \brief Class encapsulated robot status data.  The robot status data is
 * meant to mirror the industrial_msgs/RobotStatus message.
 *
 *
 * The byte representation of a robot status is as follows (in order lowest index
 * to highest). The standard sizes are given, but can change based on type sizes:
 *
 *   member:             type                                      size
 *   drives_powered      (industrial::shared_types::shared_int)    4  bytes
 *   e_stopped           (industrial::shared_types::shared_int)    4  bytes
 *   error_code          (industrial::shared_types::shared_int)    4  bytes
 *   in_error            (industrial::shared_types::shared_int)    4  bytes
 *   in_motion           (industrial::shared_types::shared_int)    4  bytes
 *   mode                (industrial::shared_types::shared_int)    4  bytes
 *   motion_possible     (industrial::shared_types::shared_int)    4  bytes
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class RobotStatus : public industrial::simple_serialize::SimpleSerialize
{
public:
/**
 * \brief Default constructor
 *
 * This method creates empty data.
 *
 */
RobotStatus(void);
/**
 * \brief Destructor
 *
 */
~RobotStatus(void);

/**
 * \brief Initializes an empty robot status
 *
 */
void init();

/**
 * \brief Initializes a full robot status message
 *
 */
void init(TriState drivesPowered, TriState eStopped, industrial::shared_types::shared_int errorCode, TriState inError,
          TriState inMotion, RobotMode mode, TriState motionPossible);

TriState getDrivesPowered()
{
  return TriState(drives_powered_);
}

TriState getEStopped()
{
  return TriState(e_stopped_);
}

industrial::shared_types::shared_int getErrorCode() const
{
  return error_code_;
}

TriState getInError()
{
  return TriState(in_error_);
}

TriState getInMotion()
{
  return TriState(in_motion_);
}

RobotMode getMode()
{
  return RobotMode(mode_);
}

TriState getMotionPossible()
{
  return TriState(motion_possible_);
}

void setDrivesPowered(TriState drivesPowered)
{
  this->drives_powered_ = drivesPowered;
}

void setEStopped(TriState eStopped)
{
  this->e_stopped_ = eStopped;
}

void setErrorCode(industrial::shared_types::shared_int errorCode)
{
  this->error_code_ = errorCode;
}

void setInError(TriState inError)
{
  this->in_error_ = inError;
}

void setInMotion(TriState inMotion)
{
  this->in_motion_ = inMotion;
}

void setMode(RobotMode mode)
{
  this->mode_ = mode;
}

void setMotionPossible(TriState motionPossible)
{
  this->motion_possible_ = motionPossible;
}

/**
 * \brief Copies the passed in value
 *
 * \param src (value to copy)
 */
void copyFrom(RobotStatus &src);

/**
 * \brief == operator implementation
 *
 * \return true if equal
 */
bool operator==(RobotStatus &rhs);

// Overrides - SimpleSerialize
bool load(industrial::byte_array::ByteArray *buffer);
bool unload(industrial::byte_array::ByteArray *buffer);
unsigned int byteLength()
{
  return 7 * sizeof(industrial::shared_types::shared_int);
}

private:

/**
 * \brief Operating mode (see RobotModes::RobotMode)
 */
industrial::shared_types::shared_int mode_;

/**
 * \brief E-stop state (see TriStates::TriState)
 */
industrial::shared_types::shared_int e_stopped_;

/**
 * \brief Drive power state (see TriStates::TriState)
 */
industrial::shared_types::shared_int drives_powered_;

/**
 * \brief motion possible state (see TriStates::TriState)
 */
industrial::shared_types::shared_int motion_possible_;

/**
 * \brief in motion state (see TriStates::TriState)
 */
industrial::shared_types::shared_int in_motion_;

/**
 * \brief in error state (see TriStates::TriState)
 */
industrial::shared_types::shared_int in_error_;

/**
 * \brief error code (non-zero is error)
 */
industrial::shared_types::shared_int error_code_;

};

}
}