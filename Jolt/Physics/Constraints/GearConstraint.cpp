// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Constraints/GearConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/ObjectStream/TypeDeclarations.h>
#include <Jolt/Core/StreamIn.h>
#include <Jolt/Core/StreamOut.h>
#ifdef JPH_DEBUG_RENDERER
	#include <Jolt/Renderer/DebugRenderer.h>
#endif // JPH_DEBUG_RENDERER

JPH_NAMESPACE_BEGIN

JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(GearConstraintSettings)
{
	JPH_ADD_BASE_CLASS(GearConstraintSettings, TwoBodyConstraintSettings)

	JPH_ADD_ENUM_ATTRIBUTE(GearConstraintSettings, mSpace)
	JPH_ADD_ATTRIBUTE(GearConstraintSettings, mHingeAxis1)
	JPH_ADD_ATTRIBUTE(GearConstraintSettings, mHingeAxis2)
	JPH_ADD_ATTRIBUTE(GearConstraintSettings, mRatio)
}

void GearConstraintSettings::SaveBinaryState(StreamOut &inStream) const
{ 
	ConstraintSettings::SaveBinaryState(inStream);

	inStream.Write(mSpace);
	inStream.Write(mHingeAxis1);
	inStream.Write(mHingeAxis2);
	inStream.Write(mRatio);
}

void GearConstraintSettings::RestoreBinaryState(StreamIn &inStream)
{
	ConstraintSettings::RestoreBinaryState(inStream);

	inStream.Read(mSpace);
	inStream.Read(mHingeAxis1);
	inStream.Read(mHingeAxis2);
	inStream.Read(mRatio);
}

TwoBodyConstraint *GearConstraintSettings::Create(Body &inBody1, Body &inBody2) const
{
	return new GearConstraint(inBody1, inBody2, *this);
}

GearConstraint::GearConstraint(Body &inBody1, Body &inBody2, const GearConstraintSettings &inSettings) :
	TwoBodyConstraint(inBody1, inBody2, inSettings),
	mLocalSpaceHingeAxis1(inSettings.mHingeAxis1),
	mLocalSpaceHingeAxis2(inSettings.mHingeAxis2),
	mRatio(inSettings.mRatio)
{
	if (inSettings.mSpace == EConstraintSpace::WorldSpace)
	{
		// If all properties were specified in world space, take them to local space now
		mLocalSpaceHingeAxis1 = inBody1.GetInverseCenterOfMassTransform().Multiply3x3(mLocalSpaceHingeAxis1).Normalized();
		mLocalSpaceHingeAxis2 = inBody2.GetInverseCenterOfMassTransform().Multiply3x3(mLocalSpaceHingeAxis2).Normalized();
	}
}

void GearConstraint::CalculateConstraintProperties(Mat44Arg inRotation1, Mat44Arg inRotation2)
{
	// Calculate world space normals
	mWorldSpaceHingeAxis1 = inRotation1 * mLocalSpaceHingeAxis1;
	mWorldSpaceHingeAxis2 = inRotation2 * mLocalSpaceHingeAxis2;

	mGearConstraintPart.CalculateConstraintProperties(*mBody1, mWorldSpaceHingeAxis1, *mBody2, mWorldSpaceHingeAxis2, mRatio);
}

void GearConstraint::SetupVelocityConstraint(decimal inDeltaTime)
{
	// Calculate constraint properties that are constant while bodies don't move
	Mat44 rotation1 = Mat44::sRotation(mBody1->GetRotation());
	Mat44 rotation2 = Mat44::sRotation(mBody2->GetRotation());
	CalculateConstraintProperties(rotation1, rotation2);
}

void GearConstraint::WarmStartVelocityConstraint(decimal inWarmStartImpulseRatio)
{
	// Warm starting: Apply previous frame impulse
	mGearConstraintPart.WarmStart(*mBody1, *mBody2, inWarmStartImpulseRatio);
}

bool GearConstraint::SolveVelocityConstraint(decimal inDeltaTime)
{
	return mGearConstraintPart.SolveVelocityConstraint(*mBody1, mWorldSpaceHingeAxis1, *mBody2, mWorldSpaceHingeAxis2, mRatio);
}

bool GearConstraint::SolvePositionConstraint(decimal inDeltaTime, decimal inBaumgarte)
{
	if (mGear1Constraint == nullptr || mGear2Constraint == nullptr)
		return false;

	decimal gear1rot;
	if (mGear1Constraint->GetSubType() == EConstraintSubType::Hinge)
	{
		gear1rot = static_cast<const HingeConstraint *>(mGear1Constraint.GetPtr())->GetCurrentAngle();
	}
	else
	{
		JPH_ASSERT(false, "Unsupported");
		return false;
	}

	decimal gear2rot;
	if (mGear2Constraint->GetSubType() == EConstraintSubType::Hinge)
	{
		gear2rot = static_cast<const HingeConstraint *>(mGear2Constraint.GetPtr())->GetCurrentAngle();
	}
	else
	{		
		JPH_ASSERT(false, "Unsupported");
		return false;
	}

	decimal error = CenterAngleAroundZero(fmod(gear1rot + mRatio * gear2rot, decimal(2.0f) * JPH_PI));
	if (error == decimal(0.0f))
		return false;

	Mat44 rotation1 = Mat44::sRotation(mBody1->GetRotation());
	Mat44 rotation2 = Mat44::sRotation(mBody2->GetRotation());
	CalculateConstraintProperties(rotation1, rotation2);
	return mGearConstraintPart.SolvePositionConstraint(*mBody1, *mBody2, error, inBaumgarte);
}

#ifdef JPH_DEBUG_RENDERER
void GearConstraint::DrawConstraint(DebugRenderer *inRenderer) const
{
	RMat44 transform1 = mBody1->GetCenterOfMassTransform();
	RMat44 transform2 = mBody2->GetCenterOfMassTransform();

	// Draw constraint axis
	inRenderer->DrawArrow(transform1.GetTranslation(), transform1 * mLocalSpaceHingeAxis1, Color::sGreen, decimal(0.01f));
	inRenderer->DrawArrow(transform2.GetTranslation(), transform2 * mLocalSpaceHingeAxis2, Color::sBlue, decimal(0.01f));
}

#endif // JPH_DEBUG_RENDERER

void GearConstraint::SaveState(StateRecorder &inStream) const
{
	TwoBodyConstraint::SaveState(inStream);

	mGearConstraintPart.SaveState(inStream);
}

void GearConstraint::RestoreState(StateRecorder &inStream)
{
	TwoBodyConstraint::RestoreState(inStream);

	mGearConstraintPart.RestoreState(inStream);
}

Ref<ConstraintSettings> GearConstraint::GetConstraintSettings() const
{
	GearConstraintSettings *settings = new GearConstraintSettings;
	ToConstraintSettings(*settings);
	settings->mSpace = EConstraintSpace::LocalToBodyCOM;
	settings->mHingeAxis1 = mLocalSpaceHingeAxis1;
	settings->mHingeAxis2 = mLocalSpaceHingeAxis2;
	settings->mRatio = mRatio;
	return settings;
}

Mat44 GearConstraint::GetConstraintToBody1Matrix() const
{
	Vec3 perp = mLocalSpaceHingeAxis1.GetNormalizedPerpendicular();
	return Mat44(Vec4(mLocalSpaceHingeAxis1, C0), Vec4(perp, C0), Vec4(mLocalSpaceHingeAxis1.Cross(perp), C0), Vec4(C0, C0, C0, C1)); 
}

Mat44 GearConstraint::GetConstraintToBody2Matrix() const 
{ 
	Vec3 perp = mLocalSpaceHingeAxis2.GetNormalizedPerpendicular();
	return Mat44(Vec4(mLocalSpaceHingeAxis2, C0), Vec4(perp, C0), Vec4(mLocalSpaceHingeAxis2.Cross(perp), C0), Vec4(C0, C0, C0, C1)); 
}

JPH_NAMESPACE_END
