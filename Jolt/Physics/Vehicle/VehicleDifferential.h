// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Jolt/ObjectStream/SerializableObject.h>
#include <Jolt/Core/StreamIn.h>
#include <Jolt/Core/StreamOut.h>

JPH_NAMESPACE_BEGIN

class VehicleDifferentialSettings
{
public:
	JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL(VehicleDifferentialSettings)

	/// Saves the contents in binary form to inStream.
	void					SaveBinaryState(StreamOut &inStream) const;

	/// Restores the contents in binary form to inStream.
	void					RestoreBinaryState(StreamIn &inStream);

	/// Calculate the torque ratio between left and right wheel
	/// @param inLeftAngularVelocity Angular velocity of left wheel (rad / s)
	/// @param inRightAngularVelocity Angular velocity of right wheel (rad / s)
	/// @param outLeftTorqueFraction Fraction of torque that should go to the left wheel
	/// @param outRightTorqueFraction Fraction of torque that should go to the right wheel
	void					CalculateTorqueRatio(decimal inLeftAngularVelocity, decimal inRightAngularVelocity, decimal &outLeftTorqueFraction, decimal &outRightTorqueFraction) const;

	int						mLeftWheel = -1;							///< Index (in mWheels) that represents the left wheel of this differential (can be -1 to indicate no wheel)
	int						mRightWheel = -1;							///< Index (in mWheels) that represents the right wheel of this differential (can be -1 to indicate no wheel)
	decimal					mDifferentialRatio = decimal(3.42f);					///< Ratio between rotation speed of gear box and wheels
	decimal					mLeftRightSplit = decimal(0.5f);						///< Defines how the engine torque is split across the left and right wheel (0 = left, 0.5 = center, 1 = right)
	decimal					mLimitedSlipRatio = decimal(1.4f);					///< Ratio max / min wheel speed. When this ratio is exceeded, all torque gets distributed to the slowest moving wheel. This allows implementing a limited slip differential. Set to FIX_MAX for an open differential. Value should be > 1.
	decimal					mEngineTorqueRatio = decimal(1.0f);					///< How much of the engines torque is applied to this differential (0 = none, 1 = full), make sure the sum of all differentials is 1.
};

JPH_NAMESPACE_END
