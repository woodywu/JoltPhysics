// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Jolt/Physics/Constraints/TwoBodyConstraint.h>
#include <Jolt/Physics/Constraints/MotorSettings.h>
#include <Jolt/Physics/Constraints/ConstraintPart/PointConstraintPart.h>
#include <Jolt/Physics/Constraints/ConstraintPart/HingeRotationConstraintPart.h>
#include <Jolt/Physics/Constraints/ConstraintPart/AngleConstraintPart.h>

JPH_NAMESPACE_BEGIN

/// Hinge constraint settings, used to create a hinge constraint
class HingeConstraintSettings final : public TwoBodyConstraintSettings
{
public:
	JPH_DECLARE_SERIALIZABLE_VIRTUAL(HingeConstraintSettings)

	// See: ConstraintSettings::SaveBinaryState
	virtual void				SaveBinaryState(StreamOut &inStream) const override;

	/// Create an an instance of this constraint
	virtual TwoBodyConstraint *	Create(Body &inBody1, Body &inBody2) const override;

	/// This determines in which space the constraint is setup, all properties below should be in the specified space
	EConstraintSpace			mSpace = EConstraintSpace::WorldSpace;

	/// Body 1 constraint reference frame (space determined by mSpace).
	/// Hinge axis is the axis where rotation is allowed, normal axis defines the 0 angle of the hinge.
	RVec3						mPoint1 = RVec3::sZero();
	Vec3						mHingeAxis1 = Vec3::sAxisY();
	Vec3						mNormalAxis1 = Vec3::sAxisX();
	
	/// Body 2 constraint reference frame (space determined by mSpace)
	RVec3						mPoint2 = RVec3::sZero();
	Vec3						mHingeAxis2 = Vec3::sAxisY();
	Vec3						mNormalAxis2 = Vec3::sAxisX();
	
	/// Bodies are assumed to be placed so that the hinge angle = 0, movement will be limited between [mLimitsMin, mLimitsMax] where mLimitsMin e [-pi, 0] and mLimitsMax e [0, pi].
	/// Both angles are in radians.
	decimal						mLimitsMin = -JPH_PI;
	decimal						mLimitsMax = JPH_PI;

	/// Maximum amount of torque (N m) to apply as friction when the constraint is not powered by a motor
	decimal						mMaxFrictionTorque = decimal(0.0f);

	/// In case the constraint is powered, this determines the motor settings around the hinge axis
	MotorSettings				mMotorSettings;

protected:
	// See: ConstraintSettings::RestoreBinaryState
	virtual void				RestoreBinaryState(StreamIn &inStream) override;
};

/// A hinge constraint constrains 2 bodies on a single point and allows only a single axis of rotation
class HingeConstraint final : public TwoBodyConstraint
{
public:
	JPH_OVERRIDE_NEW_DELETE

	/// Construct hinge constraint
								HingeConstraint(Body &inBody1, Body &inBody2, const HingeConstraintSettings &inSettings);

	// Generic interface of a constraint
	virtual EConstraintSubType	GetSubType() const override								{ return EConstraintSubType::Hinge; }
	virtual void				SetupVelocityConstraint(decimal inDeltaTime) override;
	virtual void				WarmStartVelocityConstraint(decimal inWarmStartImpulseRatio) override;
	virtual bool				SolveVelocityConstraint(decimal inDeltaTime) override;
	virtual bool				SolvePositionConstraint(decimal inDeltaTime, decimal inBaumgarte) override;
#ifdef JPH_DEBUG_RENDERER
	virtual void				DrawConstraint(DebugRenderer *inRenderer) const override;
	virtual void				DrawConstraintLimits(DebugRenderer *inRenderer) const override;
#endif // JPH_DEBUG_RENDERER
	virtual void				SaveState(StateRecorder &inStream) const override;
	virtual void				RestoreState(StateRecorder &inStream) override;
	virtual Ref<ConstraintSettings> GetConstraintSettings() const override;

	// See: TwoBodyConstraint
	virtual Mat44				GetConstraintToBody1Matrix() const override;
	virtual Mat44				GetConstraintToBody2Matrix() const override;

	/// Get the current rotation angle from the rest position
	decimal						GetCurrentAngle() const;

	// Friction control
	void						SetMaxFrictionTorque(decimal inFrictionTorque)			{ mMaxFrictionTorque = inFrictionTorque; }
	decimal						GetMaxFrictionTorque() const							{ return mMaxFrictionTorque; }

	// Motor settings
	MotorSettings &				GetMotorSettings()										{ return mMotorSettings; }
	const MotorSettings &		GetMotorSettings() const								{ return mMotorSettings; }

	// Motor controls
	void						SetMotorState(EMotorState inState)						{ JPH_ASSERT(inState == EMotorState::Off || mMotorSettings.IsValid()); mMotorState = inState; }
	EMotorState					GetMotorState() const									{ return mMotorState; }
	void						SetTargetAngularVelocity(decimal inAngularVelocity)		{ mTargetAngularVelocity = inAngularVelocity; } ///< rad/s
	decimal						GetTargetAngularVelocity() const						{ return mTargetAngularVelocity; }
	void						SetTargetAngle(decimal inAngle)							{ mTargetAngle = mHasLimits? Clamp(inAngle, mLimitsMin, mLimitsMax) : inAngle; } ///< rad
	decimal						GetTargetAngle() const									{ return mTargetAngle; }

	/// Update the rotation limits of the hinge, value in radians (see HingeConstraintSettings)
	void						SetLimits(decimal inLimitsMin, decimal inLimitsMax);
	decimal						GetLimitsMin() const									{ return mLimitsMin; }
	decimal						GetLimitsMax() const									{ return mLimitsMax; }
	bool						HasLimits() const										{ return mHasLimits; }

	///@name Get Lagrange multiplier from last physics update (relates to how much force/torque was applied to satisfy the constraint)
	inline Vec3		 			GetTotalLambdaPosition() const							{ return mPointConstraintPart.GetTotalLambda(); }
	inline Vector<2>			GetTotalLambdaRotation() const							{ return mRotationConstraintPart.GetTotalLambda(); }
	inline decimal				GetTotalLambdaRotationLimits() const					{ return mRotationLimitsConstraintPart.GetTotalLambda(); }
	inline decimal				GetTotalLambdaMotor() const								{ return mMotorConstraintPart.GetTotalLambda(); }

private:
	// Internal helper function to calculate the values below
	void						CalculateA1AndTheta();
	void						CalculateRotationLimitsConstraintProperties(decimal inDeltaTime);
	void						CalculateMotorConstraintProperties(decimal inDeltaTime);
	inline decimal				GetSmallestAngleToLimit() const;

	// CONFIGURATION PROPERTIES FOLLOW

	// Local space constraint positions
	Vec3						mLocalSpacePosition1;
	Vec3						mLocalSpacePosition2;

	// Local space hinge directions
	Vec3						mLocalSpaceHingeAxis1;
	Vec3						mLocalSpaceHingeAxis2;

	// Local space normal direction (direction relative to which to draw constraint limits)
	Vec3						mLocalSpaceNormalAxis1;
	Vec3						mLocalSpaceNormalAxis2;
		
	// Inverse of initial relative orientation between bodies (which defines hinge angle = 0)
	Quat						mInvInitialOrientation;

	// Hinge limits
	bool						mHasLimits;
	decimal						mLimitsMin;
	decimal						mLimitsMax;

	// Friction
	decimal						mMaxFrictionTorque;

	// Motor controls
	MotorSettings				mMotorSettings;
	EMotorState					mMotorState = EMotorState::Off;
	decimal						mTargetAngularVelocity = decimal(0.0f);
	decimal						mTargetAngle = decimal(0.0f);

	// RUN TIME PROPERTIES FOLLOW

	// Current rotation around the hinge axis
	decimal						mTheta = decimal(0.0f);
	
	// World space hinge axis for body 1
	Vec3						mA1;

	// The constraint parts
	PointConstraintPart			mPointConstraintPart;
	HingeRotationConstraintPart mRotationConstraintPart;
	AngleConstraintPart			mRotationLimitsConstraintPart;
	AngleConstraintPart			mMotorConstraintPart;
};

JPH_NAMESPACE_END
