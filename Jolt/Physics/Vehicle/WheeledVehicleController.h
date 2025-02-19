// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/Physics/Vehicle/VehicleController.h>
#include <Jolt/Physics/Vehicle/VehicleEngine.h>
#include <Jolt/Physics/Vehicle/VehicleTransmission.h>
#include <Jolt/Physics/Vehicle/VehicleDifferential.h>
#include <Jolt/Core/LinearCurve.h>

JPH_NAMESPACE_BEGIN

class PhysicsSystem;

/// WheelSettings object specifically for WheeledVehicleController
class WheelSettingsWV : public WheelSettings
{
public:
	JPH_DECLARE_SERIALIZABLE_VIRTUAL(WheelSettingsWV)

	/// Constructor
								WheelSettingsWV();

	// See: WheelSettings
	virtual void				SaveBinaryState(StreamOut &inStream) const override;
	virtual void				RestoreBinaryState(StreamIn &inStream) override;

	decimal						mInertia = decimal(0.9f);							///< Moment of inertia (kg m^2), for a cylinder this would be 0.5 * M * R^2 which is 0.9 for a wheel with a mass of 20 kg and radius 0.3 m
	decimal						mAngularDamping = decimal(0.2f);						///< Angular damping factor of the wheel: dw/dt = -c * w
	decimal						mMaxSteerAngle = DegreesToRadians(decimal(70.0f));	///< How much this wheel can steer (radians)
	LinearCurve					mLongitudinalFriction;						///< Friction in forward direction of tire as a function of the slip ratio (fraction): (omega_wheel * r_wheel - v_longitudinal) / |v_longitudinal|
	LinearCurve					mLateralFriction;							///< Friction in sideway direction of tire as a function of the slip angle (degrees): angle between relative contact velocity and vehicle direction
	decimal						mMaxBrakeTorque = decimal(1500.0f);					///< How much torque (Nm) the brakes can apply to this wheel
	decimal						mMaxHandBrakeTorque = decimal(4000.0f);				///< How much torque (Nm) the hand brake can apply to this wheel (usually only applied to the rear wheels)
};

/// Wheel object specifically for WheeledVehicleController
class WheelWV : public Wheel
{
public:
	JPH_OVERRIDE_NEW_DELETE

	/// Constructor
	explicit					WheelWV(const WheelSettingsWV &inWheel);

	/// Override GetSettings and cast to the correct class
	const WheelSettingsWV *		GetSettings() const							{ return static_cast<const WheelSettingsWV *>(mSettings.GetPtr()); }

	/// Apply a torque (N m) to the wheel for a particular delta time
	void						ApplyTorque(decimal inTorque, decimal inDeltaTime)
	{
		mAngularVelocity += inTorque * inDeltaTime / GetSettings()->mInertia;
	}

	/// Update the wheel rotation based on the current angular velocity
	void						Update(decimal inDeltaTime, const VehicleConstraint &inConstraint);

	decimal						mLongitudinalSlip = decimal(0.0f);					///< Velocity difference between ground and wheel relative to ground velocity
	decimal						mCombinedLongitudinalFriction = decimal(0.0f);		///< Combined friction coefficient in longitudinal direction (combines terrain and tires)
	decimal						mCombinedLateralFriction = decimal(0.0f);			///< Combined friction coefficient in lateral direction (combines terrain and tires)
	decimal						mBrakeImpulse = decimal(0.0f);						///< Amount of impulse that the brakes can apply to the floor (excluding friction)
};

/// Settings of a vehicle with regular wheels
/// 
/// The properties in this controller are largely based on "Car Physics for Games" by Marco Monster.
/// See: https://www.asawicki.info/Mirror/Car%20Physics%20for%20Games/Car%20Physics%20for%20Games.html
class WheeledVehicleControllerSettings : public VehicleControllerSettings
{
public:
	JPH_DECLARE_SERIALIZABLE_VIRTUAL(WheeledVehicleControllerSettings)

	// See: VehicleControllerSettings
	virtual VehicleController *	ConstructController(VehicleConstraint &inConstraint) const override;
	virtual void				SaveBinaryState(StreamOut &inStream) const override;
	virtual void				RestoreBinaryState(StreamIn &inStream) override;

	VehicleEngineSettings		mEngine;									///< The properties of the engine
	VehicleTransmissionSettings	mTransmission;								///< The properties of the transmission (aka gear box)
	Array<VehicleDifferentialSettings> mDifferentials;						///< List of differentials and their properties
	decimal						mDifferentialLimitedSlipRatio = decimal(1.4f);		///< Ratio max / min average wheel speed of each differential (measured at the clutch). When the ratio is exceeded all torque gets distributed to the differential with the minimal average velocity. This allows implementing a limited slip differential between differentials. Set to FIX_MAX for an open differential. Value should be > 1.
};

/// Runtime controller class
class WheeledVehicleController : public VehicleController
{
public:
	JPH_OVERRIDE_NEW_DELETE

	/// Constructor
								WheeledVehicleController(const WheeledVehicleControllerSettings &inSettings, VehicleConstraint &inConstraint);

	/// Typedefs
	using Differentials = Array<VehicleDifferentialSettings>;

	/// Set input from driver
	/// @param inForward Value between -1 and 1 for auto transmission and value between 0 and 1 indicating desired driving direction and amount the gas pedal is pressed
	/// @param inRight Value between -1 and 1 indicating desired steering angle (1 = right)
	/// @param inBrake Value between 0 and 1 indicating how strong the brake pedal is pressed
	/// @param inHandBrake Value between 0 and 1 indicating how strong the hand brake is pulled
	void						SetDriverInput(decimal inForward, decimal inRight, decimal inBrake, decimal inHandBrake) { mForwardInput = inForward; mRightInput = inRight; mBrakeInput = inBrake; mHandBrakeInput = inHandBrake; }

	/// Get current engine state
	const VehicleEngine &		GetEngine() const							{ return mEngine; }

	/// Get current engine state (writable interface, allows you to make changes to the configuration which will take effect the next time step)
	VehicleEngine &				GetEngine()									{ return mEngine; }

	/// Get current transmission state
	const VehicleTransmission &	GetTransmission() const						{ return mTransmission; }

	/// Get current transmission state (writable interface, allows you to make changes to the configuration which will take effect the next time step)
	VehicleTransmission &		GetTransmission()							{ return mTransmission; }

	/// Get the differentials this vehicle has
	const Differentials &		GetDifferentials() const					{ return mDifferentials; }

	/// Get the differentials this vehicle has (writable interface, allows you to make changes to the configuration which will take effect the next time step)
	Differentials &				GetDifferentials()							{ return mDifferentials; }

	/// Ratio max / min average wheel speed of each differential (measured at the clutch).
	decimal						GetDifferentialLimitedSlipRatio() const		{ return mDifferentialLimitedSlipRatio; }
	void						SetDifferentialLimitedSlipRatio(decimal inV)	{ mDifferentialLimitedSlipRatio = inV; }

#ifdef JPH_DEBUG_RENDERER
	/// Debug drawing of RPM meter
	void						SetRPMMeter(Vec3Arg inPosition, decimal inSize) { mRPMMeterPosition = inPosition; mRPMMeterSize = inSize; }
#endif // JPH_DEBUG_RENDERER

protected:
	// See: VehicleController
	virtual Wheel *				ConstructWheel(const WheelSettings &inWheel) const override { JPH_ASSERT(IsKindOf(&inWheel, JPH_RTTI(WheelSettingsWV))); return new WheelWV(static_cast<const WheelSettingsWV &>(inWheel)); }
	virtual bool				AllowSleep() const override					{ return mForwardInput == decimal(0.0f); }
	virtual void				PreCollide(decimal inDeltaTime, PhysicsSystem &inPhysicsSystem) override;
	virtual void				PostCollide(decimal inDeltaTime, PhysicsSystem &inPhysicsSystem) override;
	virtual bool				SolveLongitudinalAndLateralConstraints(decimal inDeltaTime) override;
	virtual void				SaveState(StateRecorder &inStream) const override;
	virtual void				RestoreState(StateRecorder &inStream) override;
#ifdef JPH_DEBUG_RENDERER
	virtual void				Draw(DebugRenderer *inRenderer) const override;
#endif // JPH_DEBUG_RENDERER

	// Control information
	decimal						mForwardInput = decimal(0.0f);						///< Value between -1 and 1 for auto transmission and value between 0 and 1 indicating desired driving direction and amount the gas pedal is pressed
	decimal						mRightInput = decimal(0.0f);							///< Value between -1 and 1 indicating desired steering angle
	decimal						mBrakeInput = decimal(0.0f);							///< Value between 0 and 1 indicating how strong the brake pedal is pressed
	decimal						mHandBrakeInput = decimal(0.0f);						///< Value between 0 and 1 indicating how strong the hand brake is pulled

	// Simluation information
	VehicleEngine				mEngine;									///< Engine state of the vehicle
	VehicleTransmission			mTransmission;								///< Transmission state of the vehicle
	Differentials				mDifferentials;								///< Differential states of the vehicle
	decimal						mDifferentialLimitedSlipRatio;				///< Ratio max / min average wheel speed of each differential (measured at the clutch).

#ifdef JPH_DEBUG_RENDERER
	// Debug settings
	Vec3						mRPMMeterPosition { C0, C1, C0 };				///< Position (in local space of the body) of the RPM meter when drawing the constraint
	decimal						mRPMMeterSize = decimal(0.5f);						///< Size of the RPM meter when drawing the constraint
#endif // JPH_DEBUG_RENDERER
};

JPH_NAMESPACE_END
