// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Vehicle/VehicleDifferential.h>
#include <Jolt/ObjectStream/TypeDeclarations.h>

JPH_NAMESPACE_BEGIN

JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(VehicleDifferentialSettings)
{
	JPH_ADD_ATTRIBUTE(VehicleDifferentialSettings, mLeftWheel)
	JPH_ADD_ATTRIBUTE(VehicleDifferentialSettings, mRightWheel)
	JPH_ADD_ATTRIBUTE(VehicleDifferentialSettings, mDifferentialRatio)
	JPH_ADD_ATTRIBUTE(VehicleDifferentialSettings, mLeftRightSplit)
	JPH_ADD_ATTRIBUTE(VehicleDifferentialSettings, mLimitedSlipRatio)
	JPH_ADD_ATTRIBUTE(VehicleDifferentialSettings, mEngineTorqueRatio)
}

void VehicleDifferentialSettings::SaveBinaryState(StreamOut &inStream) const
{
	inStream.Write(mLeftWheel);
	inStream.Write(mRightWheel);
	inStream.Write(mDifferentialRatio);
	inStream.Write(mLeftRightSplit);
	inStream.Write(mLimitedSlipRatio);
	inStream.Write(mEngineTorqueRatio);
}

void VehicleDifferentialSettings::RestoreBinaryState(StreamIn &inStream)
{
	inStream.Read(mLeftWheel);
	inStream.Read(mRightWheel);
	inStream.Read(mDifferentialRatio);
	inStream.Read(mLeftRightSplit);
	inStream.Read(mLimitedSlipRatio);
	inStream.Read(mEngineTorqueRatio);
}

void VehicleDifferentialSettings::CalculateTorqueRatio(decimal inLeftAngularVelocity, decimal inRightAngularVelocity, decimal &outLeftTorqueFraction, decimal &outRightTorqueFraction) const
{
	// Start with the default torque ratio
	outLeftTorqueFraction = decimal(1.0f) - mLeftRightSplit;
	outRightTorqueFraction = mLeftRightSplit;

	if (mLimitedSlipRatio < FIX_MAX)
	{
		JPH_ASSERT(mLimitedSlipRatio > decimal(1.0f));

		// This is a limited slip differential, adjust torque ratios according to wheel speeds
		decimal omega_l = max(decimal(1.0e-3f), abs(inLeftAngularVelocity)); // prevent div by zero by setting a minimum velocity and ignoring that the wheels may be rotating in different directions
		decimal omega_r = max(decimal(1.0e-3f), abs(inRightAngularVelocity));
		decimal omega_min = min(omega_l, omega_r);
		decimal omega_max = max(omega_l, omega_r);

		// Map into a value that is 0 when the wheels are turning at an equal rate and 1 when the wheels are turning at mLimitedSlipRotationRatio
		decimal alpha = min((omega_max / omega_min - decimal(1.0f)) / (mLimitedSlipRatio - decimal(1.0f)), decimal(1.0f));
		JPH_ASSERT(alpha >= decimal(0.0f));
		decimal one_min_alpha = decimal(1.0f) - alpha;

		if (omega_l < omega_r)
		{
			// Redirect more power to the left wheel
			outLeftTorqueFraction = outLeftTorqueFraction * one_min_alpha + alpha;
			outRightTorqueFraction = outRightTorqueFraction * one_min_alpha;
		}
		else
		{
			// Redirect more power to the right wheel
			outLeftTorqueFraction = outLeftTorqueFraction * one_min_alpha;
			outRightTorqueFraction = outRightTorqueFraction * one_min_alpha + alpha;
		}
	}

	// Assert the values add up to 1
	JPH_ASSERT(abs(outLeftTorqueFraction + outRightTorqueFraction - decimal(1.0f)) < decimal(1.0e-6f));
}

JPH_NAMESPACE_END
