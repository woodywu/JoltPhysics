// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/StateRecorder.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Core/StringTools.h>
#include <Jolt/Core/Profiler.h>
#ifdef JPH_DEBUG_RENDERER
	#include <Jolt/Renderer/DebugRenderer.h>
#endif // JPH_DEBUG_RENDERER

JPH_NAMESPACE_BEGIN

static const SphereShape sFixedToWorldShape(FIX_EPSILON);
Body Body::sFixedToWorld(false);

Body::Body(bool) :
	mPosition(Vec3::sZero()),
	mRotation(Quat::sIdentity()),
	mShape(&sFixedToWorldShape), // Dummy shape
	mFriction(C0),
	mRestitution(C0),
	mObjectLayer(cObjectLayerInvalid),
	mMotionType(EMotionType::Static)
{
	sFixedToWorldShape.SetEmbedded();
}

void Body::SetMotionType(EMotionType inMotionType)
{
	if (mMotionType == inMotionType)
		return;

	JPH_ASSERT(inMotionType == EMotionType::Static || mMotionProperties != nullptr, "Body needs to be created with mAllowDynamicOrKinematic set tot true");
	JPH_ASSERT(inMotionType != EMotionType::Static || !IsActive(), "Deactivate body first");

	// Store new motion type
	mMotionType = inMotionType;

	if (mMotionProperties != nullptr)
	{
		// Update cache
		JPH_IF_ENABLE_ASSERTS(mMotionProperties->mCachedMotionType = inMotionType;)

		switch (inMotionType)
		{
		case EMotionType::Static:
			// Stop the object
			mMotionProperties->mLinearVelocity = Vec3::sZero();
			mMotionProperties->mAngularVelocity = Vec3::sZero();
			[[fallthrough]];

		case EMotionType::Kinematic:
			// Cancel forces
			mMotionProperties->mForce = Float3(C0, C0, C0);
			mMotionProperties->mTorque = Float3(C0, C0, C0);
			break;

		case EMotionType::Dynamic:
			break;
		}
	}
}

void Body::SetAllowSleeping(bool inAllow)									
{ 
	mMotionProperties->mAllowSleeping = inAllow; 
	if (inAllow) 
		ResetSleepTestSpheres();
}

void Body::MoveKinematic(RVec3Arg inTargetPosition, QuatArg inTargetRotation, decimal inDeltaTime)
{
	JPH_ASSERT(!IsStatic());
	JPH_ASSERT(BodyAccess::sCheckRights(BodyAccess::sPositionAccess, BodyAccess::EAccess::Read)); 

	// Calculate center of mass at end situation
	RVec3 new_com = inTargetPosition + inTargetRotation * mShape->GetCenterOfMass();

	// Calculate delta position and rotation
	Vec3 delta_pos = Vec3(new_com - mPosition);
	Quat delta_rotation = inTargetRotation * mRotation.Conjugated();

	mMotionProperties->MoveKinematic(delta_pos, delta_rotation, inDeltaTime);
}

void Body::CalculateWorldSpaceBoundsInternal()
{
	mBounds = mShape->GetWorldSpaceBounds(GetCenterOfMassTransform(), Vec3::sReplicate(C1));
}

void Body::SetPositionAndRotationInternal(RVec3Arg inPosition, QuatArg inRotation) 
{ 
	JPH_ASSERT(BodyAccess::sCheckRights(BodyAccess::sPositionAccess, BodyAccess::EAccess::ReadWrite)); 

	mPosition = inPosition + inRotation * mShape->GetCenterOfMass();
	mRotation = inRotation; 

	// Initialize bounding box
	CalculateWorldSpaceBoundsInternal();

	// Reset sleeping test
	if (mMotionProperties != nullptr)
		ResetSleepTestSpheres();
}

void Body::UpdateCenterOfMassInternal(Vec3Arg inPreviousCenterOfMass, bool inUpdateMassProperties)
{
	// Update center of mass position so the world position for this body stays the same
	mPosition += mRotation * (mShape->GetCenterOfMass() - inPreviousCenterOfMass);

	// Recalculate mass and inertia if requested
	if (inUpdateMassProperties && mMotionProperties != nullptr)
		mMotionProperties->SetMassProperties(mShape->GetMassProperties());
}

void Body::SetShapeInternal(const Shape *inShape, bool inUpdateMassProperties)
{
	JPH_ASSERT(BodyAccess::sCheckRights(BodyAccess::sPositionAccess, BodyAccess::EAccess::ReadWrite)); 

	// Get the old center of mass
	Vec3 old_com = mShape->GetCenterOfMass();

	// Update the shape
	mShape = inShape;

	// Update center of mass
	UpdateCenterOfMassInternal(old_com, inUpdateMassProperties);

	// Recalculate bounding box
	CalculateWorldSpaceBoundsInternal();
}

Body::ECanSleep Body::UpdateSleepStateInternal(decimal inDeltaTime, decimal inMaxMovement, decimal inTimeBeforeSleep)
{
	// Check override & sensors will never go to sleep (they would stop detecting collisions with sleeping bodies)
	if (!mMotionProperties->mAllowSleeping || IsSensor())
		return ECanSleep::CannotSleep;

	// Get the points to test
	RVec3 points[3];
	GetSleepTestPoints(points);

#ifdef JPH_DOUBLE_PRECISION
	// Get base offset for spheres
	DVec3 offset = mMotionProperties->GetSleepTestOffset();
#endif // JPH_DOUBLE_PRECISION

	for (int i = 0; i < 3; ++i)
	{
		Sphere &sphere = mMotionProperties->mSleepTestSpheres[i];

		// Make point relative to base offset
#ifdef JPH_DOUBLE_PRECISION
		Vec3 p = Vec3(points[i] - offset);
#else
		Vec3 p = points[i];
#endif // JPH_DOUBLE_PRECISION

		// Encapsulate the point in a sphere
		sphere.EncapsulatePoint(p);

		// Test if it exceeded the max movement
		if (sphere.GetRadius() > inMaxMovement)
		{
			// Body is not sleeping, reset test
			mMotionProperties->ResetSleepTestSpheres(points);
			return ECanSleep::CannotSleep;
		}
	}

	mMotionProperties->mSleepTestTimer += inDeltaTime;
	return mMotionProperties->mSleepTestTimer >= inTimeBeforeSleep? ECanSleep::CanSleep : ECanSleep::CannotSleep;
}

bool Body::ApplyBuoyancyImpulse(RVec3Arg inSurfacePosition, Vec3Arg inSurfaceNormal, decimal inBuoyancy, decimal inLinearDrag, decimal inAngularDrag, Vec3Arg inFluidVelocity, Vec3Arg inGravity, decimal inDeltaTime)
{
	JPH_PROFILE_FUNCTION();

	// We follow the approach from 'Game Programming Gems 6' 2.5 Exact Buoyancy for Polyhedra
	// All quantities below are in world space

	// For GetSubmergedVolume we transform the surface relative to the body position for increased precision
	Mat44 rotation = Mat44::sRotation(mRotation);
	Plane surface_relative_to_body = Plane::sFromPointAndNormal(inSurfacePosition - mPosition, inSurfaceNormal);

	// Calculate amount of volume that is submerged and what the center of buoyancy is
	decimal total_volume, submerged_volume;
	Vec3 relative_center_of_buoyancy;
	mShape->GetSubmergedVolume(rotation, Vec3::sReplicate(C1), surface_relative_to_body, total_volume, submerged_volume, relative_center_of_buoyancy JPH_IF_DEBUG_RENDERER(, mPosition));
		
	// If we're not submerged, there's no point in doing the rest of the calculations
	if (submerged_volume > C0)
	{
	#ifdef JPH_DEBUG_RENDERER
		// Draw submerged volume properties
		if (Shape::sDrawSubmergedVolumes)
		{
			RVec3 center_of_buoyancy = mPosition + relative_center_of_buoyancy;
			DebugRenderer::sInstance->DrawMarker(center_of_buoyancy, Color::sWhite, C2);
			DebugRenderer::sInstance->DrawText3D(center_of_buoyancy, StringFormat("%.3f / %.3f", (double)submerged_volume, (double)total_volume));
		}
	#endif // JPH_DEBUG_RENDERER

		// When buoyancy is 1 we want neutral buoyancy, this means that the density of the liquid is the same as the density of the body at that point.
		// Buoyancy > 1 should make the object decimal, < 1 should make it sink.
		decimal inverse_mass = mMotionProperties->GetInverseMass();
		decimal fluid_density = inBuoyancy / (total_volume * inverse_mass);

		// Buoyancy force = Density of Fluid * Submerged volume * Magnitude of gravity * Up direction (eq 2.5.1)
		// Impulse = Force * Delta time
		// We should apply this at the center of buoyancy (= center of mass of submerged volume)
		Vec3 buoyancy_impulse = -fluid_density * submerged_volume * mMotionProperties->GetGravityFactor() * inGravity * inDeltaTime;

		// Calculate the velocity of the center of buoyancy relative to the fluid
		Vec3 linear_velocity = mMotionProperties->GetLinearVelocity();
		Vec3 angular_velocity = mMotionProperties->GetAngularVelocity();
		Vec3 center_of_buoyancy_velocity = linear_velocity + angular_velocity.Cross(relative_center_of_buoyancy);
		Vec3 relative_center_of_buoyancy_velocity = inFluidVelocity - center_of_buoyancy_velocity;

		// Here we deviate from the article, instead of eq 2.5.14 we use a quadratic drag formula: https://en.wikipedia.org/wiki/Drag_%28physics%29
		// Drag force = 0.5 * Fluid Density * (Velocity of fluid - Velocity of center of buoyancy)^2 * Linear Drag * Area Facing the Relative Fluid Velocity
		// Again Impulse = Force * Delta Time
		// We should apply this at the center of buoyancy (= center of mass for submerged volume with no center of mass offset)

		// Get size of local bounding box
		Vec3 size = mShape->GetLocalBounds().GetSize();

		// Determine area of the local space bounding box in the direction of the relative velocity between the fluid and the center of buoyancy
		decimal area = C0;
		decimal relative_center_of_buoyancy_velocity_len_sq = relative_center_of_buoyancy_velocity.LengthSq();
		if (relative_center_of_buoyancy_velocity_len_sq > decimal(1.0e-12f))
		{
			Vec3 local_relative_center_of_buoyancy_velocity = GetRotation().Conjugated() * relative_center_of_buoyancy_velocity;
			area = local_relative_center_of_buoyancy_velocity.Abs().Dot(size.Swizzle<SWIZZLE_Y, SWIZZLE_Z, SWIZZLE_X>() * size.Swizzle<SWIZZLE_Z, SWIZZLE_X, SWIZZLE_Y>()) / sqrt(relative_center_of_buoyancy_velocity_len_sq);
		}

		// Calculate the impulse
		Vec3 drag_impulse = (C0P5 * fluid_density * inLinearDrag * area * inDeltaTime) * relative_center_of_buoyancy_velocity * relative_center_of_buoyancy_velocity.Length();

		// Clamp magnitude against current linear velocity to prevent overshoot
		decimal linear_velocity_len_sq = linear_velocity.LengthSq();
		decimal drag_delta_linear_velocity_len_sq = (drag_impulse * inverse_mass).LengthSq();
		if (drag_delta_linear_velocity_len_sq > linear_velocity_len_sq)
			drag_impulse *= sqrt(linear_velocity_len_sq / drag_delta_linear_velocity_len_sq);

		// Calculate the resulting delta linear velocity due to buoyancy and drag
		Vec3 delta_linear_velocity = (drag_impulse + buoyancy_impulse) * inverse_mass;
		mMotionProperties->AddLinearVelocityStep(delta_linear_velocity);

		// Determine average width of the body (across the three axis)
		decimal l = (size.GetX() + size.GetY() + size.GetZ()) / C3;

		// Drag torque = -Angular Drag * Mass * Submerged volume / Total volume * (Average width of body)^2 * Angular velocity (eq 2.5.15)
		Vec3 drag_angular_impulse = (-inAngularDrag * submerged_volume / total_volume * inDeltaTime * Square(l) / inverse_mass) * angular_velocity;
		Mat44 inv_inertia = GetInverseInertia();
		Vec3 drag_delta_angular_velocity = inv_inertia * drag_angular_impulse;

		// Clamp magnitude against the current angular velocity to prevent overshoot
		decimal angular_velocity_len_sq = angular_velocity.LengthSq();
		decimal drag_delta_angular_velocity_len_sq = drag_delta_angular_velocity.LengthSq();
		if (drag_delta_angular_velocity_len_sq > angular_velocity_len_sq)
			drag_delta_angular_velocity *= sqrt(angular_velocity_len_sq / drag_delta_angular_velocity_len_sq);

		// Calculate total delta angular velocity due to drag and buoyancy
		Vec3 delta_angular_velocity = drag_delta_angular_velocity + inv_inertia * relative_center_of_buoyancy.Cross(buoyancy_impulse + drag_impulse);
		mMotionProperties->AddAngularVelocityStep(delta_angular_velocity);
		return true;
	}

	return false;
}

void Body::SaveState(StateRecorder &inStream) const
{
	// Only write properties that can change at runtime
	inStream.Write(mPosition);
	inStream.Write(mRotation);
	inStream.Write(mFriction);
	inStream.Write(mRestitution);
	mCollisionGroup.SaveBinaryState(inStream);
	inStream.Write(mMotionType);

	if (mMotionProperties != nullptr)
		mMotionProperties->SaveState(inStream);
}

void Body::RestoreState(StateRecorder &inStream)
{
	inStream.Read(mPosition);
	inStream.Read(mRotation);
	inStream.Read(mFriction);
	inStream.Read(mRestitution);
	mCollisionGroup.RestoreBinaryState(inStream);
	inStream.Read(mMotionType);

	if (mMotionProperties != nullptr)
	{
		mMotionProperties->RestoreState(inStream);
		JPH_IF_ENABLE_ASSERTS(mMotionProperties->mCachedMotionType = mMotionType);
	}

	// Initialize bounding box
	CalculateWorldSpaceBoundsInternal();
}

BodyCreationSettings Body::GetBodyCreationSettings() const
{
	BodyCreationSettings result;

	result.mPosition = GetPosition();
	result.mRotation = GetRotation();
	result.mLinearVelocity = mMotionProperties != nullptr? mMotionProperties->GetLinearVelocity() : Vec3::sZero();
	result.mAngularVelocity = mMotionProperties != nullptr? mMotionProperties->GetAngularVelocity() : Vec3::sZero();
	result.mObjectLayer = GetObjectLayer();
	result.mCollisionGroup = GetCollisionGroup();
	result.mMotionType = GetMotionType();
	result.mAllowDynamicOrKinematic = mMotionProperties != nullptr;
	result.mIsSensor = IsSensor();
	result.mUseManifoldReduction = GetUseManifoldReduction();
	result.mMotionQuality = mMotionProperties != nullptr? mMotionProperties->GetMotionQuality() : EMotionQuality::Discrete;
	result.mAllowSleeping = mMotionProperties != nullptr? GetAllowSleeping() : true;
	result.mFriction = GetFriction();
	result.mRestitution = GetRestitution();
	result.mLinearDamping = mMotionProperties != nullptr? mMotionProperties->GetLinearDamping() : C0;
	result.mAngularDamping = mMotionProperties != nullptr? mMotionProperties->GetAngularDamping() : C0;
	result.mMaxLinearVelocity = mMotionProperties != nullptr? mMotionProperties->GetMaxLinearVelocity() : C0;
	result.mMaxAngularVelocity = mMotionProperties != nullptr? mMotionProperties->GetMaxAngularVelocity() : C0;
	result.mGravityFactor = mMotionProperties != nullptr? mMotionProperties->GetGravityFactor() : C1;
	result.mOverrideMassProperties = EOverrideMassProperties::MassAndInertiaProvided;
	result.mMassPropertiesOverride.mMass = mMotionProperties != nullptr? C1 / mMotionProperties->GetInverseMassUnchecked() : FIX_MAX;
	result.mMassPropertiesOverride.mInertia = mMotionProperties != nullptr? mMotionProperties->GetLocalSpaceInverseInertiaUnchecked().Inversed3x3() : Mat44::sIdentity();
	result.SetShape(GetShape());

	return result;
}

JPH_NAMESPACE_END
