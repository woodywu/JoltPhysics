// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Jolt/Core/Reference.h>
#include <Jolt/ObjectStream/SerializableObject.h>

JPH_NAMESPACE_BEGIN

class StreamIn;
class StreamOut;

enum class EMotorState
{
	Off,																///< Motor is off
	Velocity,															///< Motor will drive to target velocity
	Position															///< Motor will drive to target position
};

/// Class that contains the settings for a constraint motor.
/// See the main page of the API documentation for more information on how to configure a motor.
class MotorSettings
{
public:
	JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL(MotorSettings)

	/// Constructor
							MotorSettings() = default;
							MotorSettings(const MotorSettings &inRHS) = default;
							MotorSettings(decimal inFrequency, decimal inDamping) : mFrequency(inFrequency), mDamping(inDamping) { JPH_ASSERT(IsValid()); }
							MotorSettings(decimal inFrequency, decimal inDamping, decimal inForceLimit, decimal inTorqueLimit) : mFrequency(inFrequency), mDamping(inDamping), mMinForceLimit(-inForceLimit), mMaxForceLimit(inForceLimit), mMinTorqueLimit(-inTorqueLimit), mMaxTorqueLimit(inTorqueLimit) { JPH_ASSERT(IsValid()); }

	/// Set asymmetric force limits
	void					SetForceLimits(decimal inMin, decimal inMax)	{ JPH_ASSERT(inMin <= inMax); mMinForceLimit = inMin; mMaxForceLimit = inMax; }

	/// Set asymmetric torque limits
	void					SetTorqueLimits(decimal inMin, decimal inMax)	{ JPH_ASSERT(inMin <= inMax); mMinTorqueLimit = inMin; mMaxTorqueLimit = inMax; }

	/// Set symmetric force limits
	void					SetForceLimit(decimal inLimit)				{ mMinForceLimit = -inLimit; mMaxForceLimit = inLimit; }

	/// Set symmetric torque limits
	void					SetTorqueLimit(decimal inLimit)				{ mMinTorqueLimit = -inLimit; mMaxTorqueLimit = inLimit; }

	/// Check if settings are valid
	bool					IsValid() const								{ return mFrequency >= decimal(0.0f) && mDamping >= decimal(0.0f) && mMinForceLimit <= mMaxForceLimit && mMinTorqueLimit <= mMaxTorqueLimit; }

	/// Saves the contents of the motor settings in binary form to inStream.
	void					SaveBinaryState(StreamOut &inStream) const;

	/// Restores contents from the binary stream inStream.
	void					RestoreBinaryState(StreamIn &inStream);

	// Settings
	decimal					mFrequency = decimal(2.0f);							///< Oscillation frequency when solving position target (Hz). Should be in the range (0, 0.5 * simulation frequency]. When simulating at 60 Hz, 20 is a good value for a strong motor. Only used for position motors.
	decimal					mDamping = decimal(1.0f);							///< Damping when solving position target (0 = minimal damping, 1 = critical damping). Only used for position motors.
	decimal					mMinForceLimit = FIX_MIN;					///< Minimum force to apply in case of a linear constraint (N). Usually this is -mMaxForceLimit unless you want a motor that can e.g. push but not pull. Not used when motor is an angular motor.
	decimal					mMaxForceLimit = FIX_MAX;					///< Maximum force to apply in case of a linear constraint (N). Not used when motor is an angular motor.
	decimal					mMinTorqueLimit = FIX_MIN;					///< Minimum torque to apply in case of a angular constraint (N m). Usually this is -mMaxTorqueLimit unless you want a motor that can e.g. push but not pull. Not used when motor is a position motor.
	decimal					mMaxTorqueLimit = FIX_MAX;					///< Maximum torque to apply in case of a angular constraint (N m). Not used when motor is a position motor.
};

JPH_NAMESPACE_END
