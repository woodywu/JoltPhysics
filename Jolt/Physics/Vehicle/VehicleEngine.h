// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Jolt/ObjectStream/SerializableObject.h>
#include <Jolt/Core/LinearCurve.h>
#include <Jolt/Core/StreamIn.h>
#include <Jolt/Core/StreamOut.h>
#include <Jolt/Physics/StateRecorder.h>

JPH_NAMESPACE_BEGIN

#ifdef JPH_DEBUG_RENDERER
	class DebugRenderer;
#endif // JPH_DEBUG_RENDERER

/// Generic properties for a vehicle engine
class VehicleEngineSettings
{
public:
	JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL(VehicleEngineSettings)

	/// Constructor
							VehicleEngineSettings();

	/// Saves the contents in binary form to inStream.
	void					SaveBinaryState(StreamOut &inStream) const;

	/// Restores the contents in binary form to inStream.
	void					RestoreBinaryState(StreamIn &inStream);

	decimal					mMaxTorque = decimal(500.0f);						///< Max amount of torque (Nm) that the engine can deliver
	decimal					mMinRPM = decimal(1000.0f);							///< Min amount of revolutions per minute (rpm) the engine can produce without stalling
	decimal					mMaxRPM = decimal(6000.0f);							///< Max amount of revolutions per minute (rpm) the engine can generate
	LinearCurve				mNormalizedTorque;							///< Curve that describes a ratio of the max torque the engine can produce vs the fraction of the max RPM of the engine
	decimal					mInertia = decimal(0.5f);							///< Moment of inertia (kg m^2) of the engine
	decimal					mAngularDamping = decimal(0.2f);						///< Angular damping factor of the wheel: dw/dt = -c * w
};

/// Runtime data for engine
class VehicleEngine : public VehicleEngineSettings
{
public:
	/// Multiply an angular velocity (rad/s) with this value to get rounds per minute (RPM)
	static constexpr decimal	cAngularVelocityToRPM = decimal(60.0f) / (decimal(2.0f) * JPH_PI);

	/// Clamp the RPM between min and max RPM
	inline void				ClampRPM()									{ mCurrentRPM = Clamp(mCurrentRPM, mMinRPM, mMaxRPM); }

	/// Current rotation speed of engine in rounds per minute
	decimal					GetCurrentRPM() const						{ return mCurrentRPM; }

	/// Update rotation speed of engine in rounds per minute
	void					SetCurrentRPM(decimal inRPM)					{ mCurrentRPM = inRPM; ClampRPM(); }

	/// Get current angular velocity of the engine in radians / second
	inline decimal			GetAngularVelocity() const					{ return mCurrentRPM / cAngularVelocityToRPM; }

	/// Get the amount of torque (N m) that the engine can supply
	/// @param inAcceleration How much the gas pedal is pressed [0, 1]
	decimal					GetTorque(decimal inAcceleration) const		{ return inAcceleration * mMaxTorque * mNormalizedTorque.GetValue(mCurrentRPM / mMaxRPM); }

	/// Apply a torque to the engine rotation speed
	/// @param inTorque Torque in N m
	/// @param inDeltaTime Delta time in seconds
	void					ApplyTorque(decimal inTorque, decimal inDeltaTime);

	/// Update the engine RPM for damping
	/// @param inDeltaTime Delta time in seconds
	void					ApplyDamping(decimal inDeltaTime);

#ifdef JPH_DEBUG_RENDERER
	/// Debug draw a RPM meter
	void					DrawRPM(DebugRenderer *inRenderer, RVec3Arg inPosition, Vec3Arg inForward, Vec3Arg inUp, decimal inSize, decimal inShiftDownRPM, decimal inShiftUpRPM) const;
#endif // JPH_DEBUG_RENDERER

	/// Saving state for replay
	void					SaveState(StateRecorder &inStream) const;
	void					RestoreState(StateRecorder &inStream);

private:
	decimal					mCurrentRPM = mMinRPM;						///< Current rotation speed of engine in rounds per minute
};

JPH_NAMESPACE_END
