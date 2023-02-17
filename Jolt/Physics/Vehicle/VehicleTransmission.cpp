// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Vehicle/VehicleTransmission.h>
#include <Jolt/ObjectStream/TypeDeclarations.h>

JPH_NAMESPACE_BEGIN

JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(VehicleTransmissionSettings)
{
	JPH_ADD_ENUM_ATTRIBUTE(VehicleTransmissionSettings, mMode)
	JPH_ADD_ATTRIBUTE(VehicleTransmissionSettings, mGearRatios)
	JPH_ADD_ATTRIBUTE(VehicleTransmissionSettings, mReverseGearRatios)
	JPH_ADD_ATTRIBUTE(VehicleTransmissionSettings, mSwitchTime)
	JPH_ADD_ATTRIBUTE(VehicleTransmissionSettings, mClutchReleaseTime)
	JPH_ADD_ATTRIBUTE(VehicleTransmissionSettings, mSwitchLatency)
	JPH_ADD_ATTRIBUTE(VehicleTransmissionSettings, mShiftUpRPM)
	JPH_ADD_ATTRIBUTE(VehicleTransmissionSettings, mShiftDownRPM)
	JPH_ADD_ATTRIBUTE(VehicleTransmissionSettings, mClutchStrength)
}

void VehicleTransmissionSettings::SaveBinaryState(StreamOut &inStream) const
{
	inStream.Write(mMode);
	inStream.Write(mGearRatios);
	inStream.Write(mReverseGearRatios);
	inStream.Write(mSwitchTime);
	inStream.Write(mClutchReleaseTime);
	inStream.Write(mSwitchLatency);
	inStream.Write(mShiftUpRPM);
	inStream.Write(mShiftDownRPM);
	inStream.Write(mClutchStrength);
}

void VehicleTransmissionSettings::RestoreBinaryState(StreamIn &inStream)
{
	inStream.Read(mMode);
	inStream.Read(mGearRatios);
	inStream.Read(mReverseGearRatios);
	inStream.Read(mSwitchTime);
	inStream.Read(mClutchReleaseTime);
	inStream.Read(mSwitchLatency);
	inStream.Read(mShiftUpRPM);
	inStream.Read(mShiftDownRPM);
	inStream.Read(mClutchStrength);
}

void VehicleTransmission::Update(decimal inDeltaTime, decimal inCurrentRPM, decimal inForwardInput, bool inCanShiftUp)
{
	// Update current gear and calculate clutch friction
	if (mMode == ETransmissionMode::Auto)
	{
		// Switch gears based on rpm
		int old_gear = mCurrentGear;
		if (mCurrentGear == 0 // In neutral
			|| inForwardInput * decimal(mCurrentGear) < decimal(0.0f)) // Changing between forward / reverse
		{
			// Switch to first gear or reverse depending on input
			mCurrentGear = inForwardInput > decimal(0.0f)? 1 : (inForwardInput < decimal(0.0f)? -1 : 0);
		}
		else if (mGearSwitchLatencyTimeLeft == decimal(0.0f)) // If not in the timout after switching gears
		{
			if (inCanShiftUp && inCurrentRPM > mShiftUpRPM)
			{
				if (mCurrentGear < 0)
				{
					// Shift up, reverse
					if (mCurrentGear > -(int)mReverseGearRatios.size())
						mCurrentGear--;
				}
				else
				{
					// Shift up, forward
					if (mCurrentGear < (int)mGearRatios.size())
						mCurrentGear++;
				}
			}
			else if (inCurrentRPM < mShiftDownRPM)
			{
				if (mCurrentGear < 0)
				{
					// Shift down, reverse
					int max_gear = inForwardInput != decimal(0.0f)? -1 : 0;
					if (mCurrentGear < max_gear)
						mCurrentGear++;
				}
				else
				{
					// Shift down, forward
					int min_gear = inForwardInput != decimal(0.0f)? 1 : 0;
					if (mCurrentGear > min_gear)
						mCurrentGear--;
				}
			}
		}

		if (old_gear != mCurrentGear)
		{
			// We've shifted gear, start switch countdown
			mGearSwitchTimeLeft = old_gear != 0? mSwitchTime : decimal(0.0f);
			mClutchReleaseTimeLeft = mClutchReleaseTime;
			mGearSwitchLatencyTimeLeft = mSwitchLatency;
			mClutchFriction = decimal(0.0f);
		}
		else if (mGearSwitchTimeLeft > decimal(0.0f))
		{
			// If still switching gears, count down
			mGearSwitchTimeLeft = max(decimal(0.0f), mGearSwitchTimeLeft - inDeltaTime);
			mClutchFriction = decimal(0.0f);
		}
		else if (mClutchReleaseTimeLeft > decimal(0.0f))
		{
			// After switching the gears we slowly release the clutch
			mClutchReleaseTimeLeft = max(decimal(0.0f), mClutchReleaseTimeLeft - inDeltaTime);
			mClutchFriction = decimal(1.0f) - mClutchReleaseTimeLeft / mClutchReleaseTime;
		}
		else
		{
			// Clutch has full friction
			mClutchFriction = decimal(1.0f);

			// Count down switch latency
			mGearSwitchLatencyTimeLeft = max(decimal(0.0f), mGearSwitchLatencyTimeLeft - inDeltaTime);
		}
	}
}

decimal VehicleTransmission::GetCurrentRatio() const
{
	if (mCurrentGear < 0)
		return mReverseGearRatios[-mCurrentGear - 1];
	else if (mCurrentGear == 0)
		return decimal(0.0f);
	else
		return mGearRatios[mCurrentGear - 1];
}

void VehicleTransmission::SaveState(StateRecorder &inStream) const
{
	inStream.Write(mCurrentGear);
	inStream.Write(mClutchFriction);
	inStream.Write(mGearSwitchTimeLeft);
	inStream.Write(mClutchReleaseTimeLeft);
	inStream.Write(mGearSwitchLatencyTimeLeft);
}

void VehicleTransmission::RestoreState(StateRecorder &inStream)
{
	inStream.Read(mCurrentGear);
	inStream.Read(mClutchFriction);
	inStream.Read(mGearSwitchTimeLeft);
	inStream.Read(mClutchReleaseTimeLeft);
	inStream.Read(mGearSwitchLatencyTimeLeft);
}

JPH_NAMESPACE_END
