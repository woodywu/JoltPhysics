// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Jolt/ObjectStream/SerializableObject.h>
#include <Jolt/Core/StreamIn.h>
#include <Jolt/Core/StreamOut.h>
#include <Jolt/Physics/StateRecorder.h>

JPH_NAMESPACE_BEGIN

/// How gears are shifted
enum class ETransmissionMode : uint8
{
	Auto,																///< Automatically shift gear up and down
	Manual,																///< Manual gear shift (call SetTransmissionInput)
};

/// Configuration for the transmission of a vehicle (gear box)
class VehicleTransmissionSettings
{
public:
	JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL(VehicleTransmissionSettings)

	/// Saves the contents in binary form to inStream.
	void					SaveBinaryState(StreamOut &inStream) const;

	/// Restores the contents in binary form to inStream.
	void					RestoreBinaryState(StreamIn &inStream);

	ETransmissionMode		mMode = ETransmissionMode::Auto;			///< How to switch gears
	Array<decimal>			mGearRatios { decimal(2.66f), decimal(1.78f), decimal(1.3f), decimal(1.0f), decimal(0.74f) }; ///< Ratio in rotation rate between engine and gear box, first element is 1st gear, 2nd element 2nd gear etc.
	Array<decimal>			mReverseGearRatios { -decimal(2.90f) };				///< Ratio in rotation rate between engine and gear box when driving in reverse
	decimal					mSwitchTime = decimal(0.5f);							///< How long it takes to switch gears (s), only used in auto mode
	decimal					mClutchReleaseTime = decimal(0.3f);					///< How long it takes to release the clutch (go to full friction), only used in auto mode
	decimal					mSwitchLatency = decimal(0.5f);						///< How long to wait after releasing the clutch before another switch is attempted (s), only used in auto mode
	decimal					mShiftUpRPM = decimal(4000.0f);						///< If RPM of engine is bigger then this we will shift a gear up, only used in auto mode
	decimal					mShiftDownRPM = decimal(2000.0f);					///< If RPM of engine is smaller then this we will shift a gear down, only used in auto mode
	decimal					mClutchStrength = decimal(10.0f);					///< Strength of the clutch when fully engaged. Total torque a clutch applies is Torque = ClutchStrength * (Velocity Engine - Avg Velocity Wheels) (units: k m^2 s^-1)
};

/// Runtime data for transmission
class VehicleTransmission : public VehicleTransmissionSettings
{
public:
	/// Set input from driver regarding the transmission (only relevant when transmission is set to manual mode)
	/// @param inCurrentGear Current gear, -1 = reverse, 0 = neutral, 1 = 1st gear etc.
	/// @param inClutchFriction Value between 0 and 1 indicating how much friction the clutch gives (0 = no friction, 1 = full friction)
	void					Set(int inCurrentGear, decimal inClutchFriction) { mCurrentGear = inCurrentGear; mClutchFriction = inClutchFriction; }

	/// Update the current gear and clutch friction if the transmission is in aut mode
	/// @param inDeltaTime Time step delta time in s
	/// @param inCurrentRPM Current RPM for engine
	/// @param inForwardInput Hint if the user wants to drive forward (> 0) or backwards (< 0)
	/// @param inCanShiftUp Indicates if we want to allow the transmission to shift up (e.g. pass false if wheels are slipping)
	void					Update(decimal inDeltaTime, decimal inCurrentRPM, decimal inForwardInput, bool inCanShiftUp);

	/// Current gear, -1 = reverse, 0 = neutral, 1 = 1st gear etc.
	int						GetCurrentGear() const						{ return mCurrentGear; }

	/// Value between 0 and 1 indicating how much friction the clutch gives (0 = no friction, 1 = full friction)
	decimal					GetClutchFriction() const					{ return mClutchFriction; }

	/// If the auto box is currently switching gears
	bool					IsSwitchingGear() const						{ return mGearSwitchTimeLeft > decimal(0.0f); }

	/// Return the transmission ratio based on the current gear (ratio between engine and differential)
	decimal					GetCurrentRatio() const;

	/// Saving state for replay
	void					SaveState(StateRecorder &inStream) const;
	void					RestoreState(StateRecorder &inStream);

private:
	int						mCurrentGear = 0;							///< Current gear, -1 = reverse, 0 = neutral, 1 = 1st gear etc.
	decimal					mClutchFriction = decimal(1.0f);						///< Value between 0 and 1 indicating how much friction the clutch gives (0 = no friction, 1 = full friction)
	decimal					mGearSwitchTimeLeft = decimal(0.0f);					///< When switching gears this will be > 0 and will cause the engine to not provide any torque to the wheels for a short time (used for automatic gear switching only)
	decimal					mClutchReleaseTimeLeft = decimal(0.0f);				///< After switching gears this will be > 0 and will cause the clutch friction to go from 0 to 1 (used for automatic gear switching only)
	decimal					mGearSwitchLatencyTimeLeft = decimal(0.0f);			///< After releasing the clutch this will be > 0 and will prevent another gear switch (used for automatic gear switching only)
};

JPH_NAMESPACE_END
