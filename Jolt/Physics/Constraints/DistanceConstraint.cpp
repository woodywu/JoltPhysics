// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/ObjectStream/TypeDeclarations.h>
#include <Jolt/Core/StreamIn.h>
#include <Jolt/Core/StreamOut.h>
#ifdef JPH_DEBUG_RENDERER
	#include <Jolt/Renderer/DebugRenderer.h>
#endif // JPH_DEBUG_RENDERER

JPH_NAMESPACE_BEGIN

using namespace literals;

JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(DistanceConstraintSettings)
{
	JPH_ADD_BASE_CLASS(DistanceConstraintSettings, TwoBodyConstraintSettings)

	JPH_ADD_ENUM_ATTRIBUTE(DistanceConstraintSettings, mSpace)
	JPH_ADD_ATTRIBUTE(DistanceConstraintSettings, mPoint1)
	JPH_ADD_ATTRIBUTE(DistanceConstraintSettings, mPoint2)
	JPH_ADD_ATTRIBUTE(DistanceConstraintSettings, mMinDistance)
	JPH_ADD_ATTRIBUTE(DistanceConstraintSettings, mMaxDistance)
	JPH_ADD_ATTRIBUTE(DistanceConstraintSettings, mFrequency)
	JPH_ADD_ATTRIBUTE(DistanceConstraintSettings, mDamping)
}

void DistanceConstraintSettings::SaveBinaryState(StreamOut &inStream) const
{ 
	ConstraintSettings::SaveBinaryState(inStream);

	inStream.Write(mSpace);
	inStream.Write(mPoint1);
	inStream.Write(mPoint2);
	inStream.Write(mMinDistance);
	inStream.Write(mMaxDistance);
	inStream.Write(mFrequency);
	inStream.Write(mDamping);
}

void DistanceConstraintSettings::RestoreBinaryState(StreamIn &inStream)
{
	ConstraintSettings::RestoreBinaryState(inStream);

	inStream.Read(mSpace);
	inStream.Read(mPoint1);
	inStream.Read(mPoint2);
	inStream.Read(mMinDistance);
	inStream.Read(mMaxDistance);
	inStream.Read(mFrequency);
	inStream.Read(mDamping);
}

TwoBodyConstraint *DistanceConstraintSettings::Create(Body &inBody1, Body &inBody2) const
{
	return new DistanceConstraint(inBody1, inBody2, *this);
}

DistanceConstraint::DistanceConstraint(Body &inBody1, Body &inBody2, const DistanceConstraintSettings &inSettings) :
	TwoBodyConstraint(inBody1, inBody2, inSettings),
	mMinDistance(inSettings.mMinDistance),
	mMaxDistance(inSettings.mMaxDistance)
{
	if (inSettings.mSpace == EConstraintSpace::WorldSpace)
	{
		// If all properties were specified in world space, take them to local space now
		mLocalSpacePosition1 = Vec3(inBody1.GetInverseCenterOfMassTransform() * inSettings.mPoint1);
		mLocalSpacePosition2 = Vec3(inBody2.GetInverseCenterOfMassTransform() * inSettings.mPoint2);
		mWorldSpacePosition1 = inSettings.mPoint1;
		mWorldSpacePosition2 = inSettings.mPoint2;
	}
	else
	{
		// If properties were specified in local space, we need to calculate world space positions
		mLocalSpacePosition1 = Vec3(inSettings.mPoint1);
		mLocalSpacePosition2 = Vec3(inSettings.mPoint2);
		mWorldSpacePosition1 = inBody1.GetCenterOfMassTransform() * inSettings.mPoint1;
		mWorldSpacePosition2 = inBody2.GetCenterOfMassTransform() * inSettings.mPoint2;
	}

	// Store distance we want to keep between the world space points
	decimal distance = Vec3(mWorldSpacePosition2 - mWorldSpacePosition1).Length();
	SetDistance(mMinDistance < decimal(0.0f)? distance : mMinDistance, mMaxDistance < decimal(0.0f)? distance : mMaxDistance);

	// Most likely gravity is going to tear us apart (this is only used when the distance between the points = 0)
	mWorldSpaceNormal = Vec3::sAxisY(); 

	// Store frequency and damping
	SetFrequency(inSettings.mFrequency);
	SetDamping(inSettings.mDamping);
}

void DistanceConstraint::CalculateConstraintProperties(decimal inDeltaTime)
{
	// Update world space positions (the bodies may have moved)
	mWorldSpacePosition1 = mBody1->GetCenterOfMassTransform() * mLocalSpacePosition1;
	mWorldSpacePosition2 = mBody2->GetCenterOfMassTransform() * mLocalSpacePosition2;

	// Calculate world space normal
	Vec3 delta = Vec3(mWorldSpacePosition2 - mWorldSpacePosition1);
	decimal delta_len = delta.Length();
	if (delta_len > decimal(0.0f))
		mWorldSpaceNormal = delta / delta_len;

	// Calculate points relative to body
	// r1 + u = (p1 - x1) + (p2 - p1) = p2 - x1
	Vec3 r1_plus_u = Vec3(mWorldSpacePosition2 - mBody1->GetCenterOfMassPosition());
	Vec3 r2 = Vec3(mWorldSpacePosition2 - mBody2->GetCenterOfMassPosition());

	if (mMinDistance == mMaxDistance)
	{
		mAxisConstraint.CalculateConstraintProperties(inDeltaTime, *mBody1, r1_plus_u, *mBody2, r2, mWorldSpaceNormal, decimal(0.0f), delta_len - mMinDistance, mFrequency, mDamping);

		// Single distance, allow constraint forces in both directions
		mMinLambda = FIX_MIN;
		mMaxLambda = FIX_MAX;
	}
	if (delta_len <= mMinDistance)
	{
		mAxisConstraint.CalculateConstraintProperties(inDeltaTime, *mBody1, r1_plus_u, *mBody2, r2, mWorldSpaceNormal, decimal(0.0f), delta_len - mMinDistance, mFrequency, mDamping);

		// Allow constraint forces to make distance bigger only
		mMinLambda = C0;
		mMaxLambda = FIX_MAX;
	}
	else if (delta_len >= mMaxDistance)
	{
		mAxisConstraint.CalculateConstraintProperties(inDeltaTime, *mBody1, r1_plus_u, *mBody2, r2, mWorldSpaceNormal, decimal(0.0f), delta_len - mMaxDistance, mFrequency, mDamping);

		// Allow constraint forces to make distance smaller only
		mMinLambda = FIX_MIN;
		mMaxLambda = C0;
	}
	else
		mAxisConstraint.Deactivate();
}

void DistanceConstraint::SetupVelocityConstraint(decimal inDeltaTime)
{
	CalculateConstraintProperties(inDeltaTime);
}

void DistanceConstraint::WarmStartVelocityConstraint(decimal inWarmStartImpulseRatio)
{
	mAxisConstraint.WarmStart(*mBody1, *mBody2, mWorldSpaceNormal, inWarmStartImpulseRatio);
}

bool DistanceConstraint::SolveVelocityConstraint(decimal inDeltaTime)
{
	if (mAxisConstraint.IsActive())
		return mAxisConstraint.SolveVelocityConstraint(*mBody1, *mBody2, mWorldSpaceNormal, mMinLambda, mMaxLambda);
	else
		return false;
}

bool DistanceConstraint::SolvePositionConstraint(decimal inDeltaTime, decimal inBaumgarte)
{
	decimal distance = Vec3(mWorldSpacePosition2 - mWorldSpacePosition1).Dot(mWorldSpaceNormal);

	// Calculate position error
	decimal position_error = decimal(0.0f);
	if (distance < mMinDistance)
		position_error = distance - mMinDistance;
	else if (distance > mMaxDistance)
		position_error = distance - mMaxDistance;

	if (position_error != decimal(0.0f))
	{
		// Update constraint properties (bodies may have moved)
		CalculateConstraintProperties(inDeltaTime);

		return mAxisConstraint.SolvePositionConstraint(*mBody1, *mBody2, mWorldSpaceNormal, position_error, inBaumgarte);
	}

	return false;
}

#ifdef JPH_DEBUG_RENDERER
void DistanceConstraint::DrawConstraint(DebugRenderer *inRenderer) const
{
	// Draw constraint
	Vec3 delta = Vec3(mWorldSpacePosition2 - mWorldSpacePosition1);
	decimal len = delta.Length();
	if (len < mMinDistance)
	{
		RVec3 real_end_pos = mWorldSpacePosition1 + (len > decimal(0.0f)? delta * mMinDistance / len : Vec3(C0, len, C0));
		inRenderer->DrawLine(mWorldSpacePosition1, mWorldSpacePosition2, Color::sGreen);
		inRenderer->DrawLine(mWorldSpacePosition2, real_end_pos, Color::sYellow);
	}
	else if (len > mMaxDistance)
	{
		RVec3 real_end_pos = mWorldSpacePosition1 + (len > decimal(0.0f)? delta * mMaxDistance / len : Vec3(C0, len, C0));
		inRenderer->DrawLine(mWorldSpacePosition1, real_end_pos, Color::sGreen);
		inRenderer->DrawLine(real_end_pos, mWorldSpacePosition2, Color::sRed);
	}
	else
		inRenderer->DrawLine(mWorldSpacePosition1, mWorldSpacePosition2, Color::sGreen);

	// Draw constraint end points
	inRenderer->DrawMarker(mWorldSpacePosition1, Color::sWhite, decimal(0.1f));
	inRenderer->DrawMarker(mWorldSpacePosition2, Color::sWhite, decimal(0.1f));

	// Draw current length
	inRenderer->DrawText3D(0.5_r * (mWorldSpacePosition1 + mWorldSpacePosition2), StringFormat("%.2f", (double)len));
}
#endif // JPH_DEBUG_RENDERER

void DistanceConstraint::SaveState(StateRecorder &inStream) const
{
	TwoBodyConstraint::SaveState(inStream);

	mAxisConstraint.SaveState(inStream);
	inStream.Write(mWorldSpaceNormal); // When distance = 0, the normal is used from last frame so we need to store it
}

void DistanceConstraint::RestoreState(StateRecorder &inStream)
{
	TwoBodyConstraint::RestoreState(inStream);

	mAxisConstraint.RestoreState(inStream);
	inStream.Read(mWorldSpaceNormal);
}

Ref<ConstraintSettings> DistanceConstraint::GetConstraintSettings() const
{
	DistanceConstraintSettings *settings = new DistanceConstraintSettings;
	ToConstraintSettings(*settings);
	settings->mSpace = EConstraintSpace::LocalToBodyCOM;
	settings->mPoint1 = RVec3(mLocalSpacePosition1);
	settings->mPoint2 = RVec3(mLocalSpacePosition2);
	settings->mMinDistance = mMinDistance;
	settings->mMaxDistance = mMaxDistance;
	settings->mFrequency = mFrequency;
	settings->mDamping = mDamping;
	return settings;
}

JPH_NAMESPACE_END
