// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Collision/Shape/TaperedCapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/ScaleHelpers.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/Geometry/RayCapsule.h>
#include <Jolt/ObjectStream/TypeDeclarations.h>
#include <Jolt/Core/StreamIn.h>
#include <Jolt/Core/StreamOut.h>
#ifdef JPH_DEBUG_RENDERER
	#include <Jolt/Renderer/DebugRenderer.h>
#endif // JPH_DEBUG_RENDERER

JPH_NAMESPACE_BEGIN

JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(TaperedCapsuleShapeSettings)
{
	JPH_ADD_BASE_CLASS(TaperedCapsuleShapeSettings, ConvexShapeSettings)

	JPH_ADD_ATTRIBUTE(TaperedCapsuleShapeSettings, mHalfHeightOfTaperedCylinder)
	JPH_ADD_ATTRIBUTE(TaperedCapsuleShapeSettings, mTopRadius)
	JPH_ADD_ATTRIBUTE(TaperedCapsuleShapeSettings, mBottomRadius)
}

bool TaperedCapsuleShapeSettings::IsSphere() const
{
	return max(mTopRadius, mBottomRadius) >= C2 * mHalfHeightOfTaperedCylinder + min(mTopRadius, mBottomRadius);
}

ShapeSettings::ShapeResult TaperedCapsuleShapeSettings::Create() const
{ 
	if (mCachedResult.IsEmpty())
	{
		Ref<Shape> shape;
		if (IsValid() && IsSphere())
		{
			// Determine sphere center and radius
			decimal radius, center;
			if (mTopRadius > mBottomRadius)
			{
				radius = mTopRadius;
				center = mHalfHeightOfTaperedCylinder;
			}
			else
			{
				radius = mBottomRadius;
				center = -mHalfHeightOfTaperedCylinder;
			}

			// Create sphere
			shape = new SphereShape(radius, mMaterial);

			// Offset sphere if needed
			if (abs(center) > decimal(1.0e-6f))
			{
				RotatedTranslatedShapeSettings rot_trans(Vec3(C0, center, C0), Quat::sIdentity(), shape);
				mCachedResult = rot_trans.Create();
			}
			else
				mCachedResult.Set(shape);
		}
		else
		{
			// Normal tapered capsule shape
			shape = new TaperedCapsuleShape(*this, mCachedResult); 
		}
	}
	return mCachedResult;
}

TaperedCapsuleShapeSettings::TaperedCapsuleShapeSettings(decimal inHalfHeightOfTaperedCylinder, decimal inTopRadius, decimal inBottomRadius, const PhysicsMaterial *inMaterial) : 
	ConvexShapeSettings(inMaterial),
	mHalfHeightOfTaperedCylinder(inHalfHeightOfTaperedCylinder),
	mTopRadius(inTopRadius), 
	mBottomRadius(inBottomRadius)
{ 
}

TaperedCapsuleShape::TaperedCapsuleShape(const TaperedCapsuleShapeSettings &inSettings, ShapeResult &outResult) :
	ConvexShape(EShapeSubType::TaperedCapsule, inSettings, outResult),
	mTopRadius(inSettings.mTopRadius), 
	mBottomRadius(inSettings.mBottomRadius)
{ 
	if (mTopRadius <= C0)
	{
		outResult.SetError("Invalid top radius");
		return;
	}

	if (mBottomRadius <= C0)
	{
		outResult.SetError("Invalid bottom radius");
		return;
	}

	if (inSettings.mHalfHeightOfTaperedCylinder <= C0)
	{
		outResult.SetError("Invalid height");
		return;
	}

	// If this goes off one of the sphere ends falls totally inside the other and you should use a sphere instead
	if (inSettings.IsSphere())
	{
		outResult.SetError("One sphere embedded in other sphere, please use sphere shape instead");
		return;
	}

	// Approximation: The center of mass is exactly half way between the top and bottom cap of the tapered capsule
	mTopCenter = inSettings.mHalfHeightOfTaperedCylinder + C0P5 * (mBottomRadius - mTopRadius);
	mBottomCenter = -inSettings.mHalfHeightOfTaperedCylinder + C0P5 * (mBottomRadius - mTopRadius);

	// Calculate center of mass
	mCenterOfMass = Vec3(C0, inSettings.mHalfHeightOfTaperedCylinder - mTopCenter, C0);

	// Calculate convex radius
	mConvexRadius = min(mTopRadius, mBottomRadius);
	JPH_ASSERT(mConvexRadius > C0);

	// Calculate the sin and tan of the angle that the cone surface makes with the Y axis
	// See: TaperedCapsuleShape.gliffy
	mSinAlpha = (mBottomRadius - mTopRadius) / (mTopCenter - mBottomCenter);
	JPH_ASSERT(mSinAlpha >= -C1 && mSinAlpha <= C1);
	mTanAlpha = Tan(ASin(mSinAlpha));

	outResult.Set(this);
}

class TaperedCapsuleShape::TaperedCapsule final : public Support
{
public:
					TaperedCapsule(Vec3Arg inTopCenter, Vec3Arg inBottomCenter, decimal inTopRadius, decimal inBottomRadius, decimal inConvexRadius) : 
		mTopCenter(inTopCenter),
		mBottomCenter(inBottomCenter),
		mTopRadius(inTopRadius),
		mBottomRadius(inBottomRadius),
		mConvexRadius(inConvexRadius)
	{ 
		static_assert(sizeof(TaperedCapsule) <= sizeof(SupportBuffer), "Buffer size too small"); 
		JPH_ASSERT(IsAligned(this, alignof(TaperedCapsule)));
	}

	virtual Vec3	GetSupport(Vec3Arg inDirection) const override
	{ 
		// Check zero vector
		decimal len = inDirection.Length();
		if (len == C0)
			return mTopCenter + Vec3(C0, mTopRadius, C0); // Return top

		// Check if the support of the top sphere or bottom sphere is bigger
		Vec3 support_top = mTopCenter + (mTopRadius / len) * inDirection;
		Vec3 support_bottom = mBottomCenter + (mBottomRadius / len) * inDirection;
		if (support_top.Dot(inDirection) > support_bottom.Dot(inDirection))
			return support_top;
		else
			return support_bottom;
	}

	virtual decimal	GetConvexRadius() const override
	{
		return mConvexRadius;
	}

private:
	Vec3			mTopCenter;
	Vec3			mBottomCenter;
	decimal			mTopRadius;
	decimal			mBottomRadius;
	decimal			mConvexRadius;
};

const ConvexShape::Support *TaperedCapsuleShape::GetSupportFunction(ESupportMode inMode, SupportBuffer &inBuffer, Vec3Arg inScale) const
{
	JPH_ASSERT(IsValidScale(inScale));

	// Get scaled tapered capsule
	Vec3 abs_scale = inScale.Abs();
	decimal scale_xz = abs_scale.GetX();
	decimal scale_y = inScale.GetY(); // The sign of y is important as it flips the tapered capsule
	Vec3 scaled_top_center = Vec3(C0, scale_y * mTopCenter, C0);
	Vec3 scaled_bottom_center = Vec3(C0, scale_y * mBottomCenter, C0);
	decimal scaled_top_radius = scale_xz * mTopRadius;
	decimal scaled_bottom_radius = scale_xz * mBottomRadius;
	decimal scaled_convex_radius = scale_xz * mConvexRadius;

	switch (inMode)
	{
	case ESupportMode::IncludeConvexRadius:
		return new (&inBuffer) TaperedCapsule(scaled_top_center, scaled_bottom_center, scaled_top_radius, scaled_bottom_radius, C0);

	case ESupportMode::ExcludeConvexRadius:
		{
			// Get radii reduced by convex radius
			decimal tr = scaled_top_radius - scaled_convex_radius;
			decimal br = scaled_bottom_radius - scaled_convex_radius;
			JPH_ASSERT(tr >= C0 && br >= C0);
			JPH_ASSERT(tr == C0 || br == C0, "Convex radius should be that of the smallest sphere");
			return new (&inBuffer) TaperedCapsule(scaled_top_center, scaled_bottom_center, tr, br, scaled_convex_radius);
		}
	}

	JPH_ASSERT(false);
	return nullptr;
}

void TaperedCapsuleShape::GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const
{	
	JPH_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID");
	JPH_ASSERT(IsValidScale(inScale));

	// Check zero vector
	decimal len = inDirection.Length();
	if (len == C0)
		return;

	// Get scaled tapered capsule
	Vec3 abs_scale = inScale.Abs();
	decimal scale_xz = abs_scale.GetX();
	decimal scale_y = inScale.GetY(); // The sign of y is important as it flips the tapered capsule
	Vec3 scaled_top_center = Vec3(C0, scale_y * mTopCenter, C0);
	Vec3 scaled_bottom_center = Vec3(C0, scale_y * mBottomCenter, C0);
	decimal scaled_top_radius = scale_xz * mTopRadius;
	decimal scaled_bottom_radius = scale_xz * mBottomRadius;

	// Get support point for top and bottom sphere in the opposite of inDirection (including convex radius)
	Vec3 support_top = scaled_top_center - (scaled_top_radius / len) * inDirection;
	Vec3 support_bottom = scaled_bottom_center - (scaled_bottom_radius / len) * inDirection;

	// Get projection on inDirection
	decimal proj_top = support_top.Dot(inDirection);
	decimal proj_bottom = support_bottom.Dot(inDirection);

	// If projection is roughly equal then return line, otherwise we return nothing as there's only 1 point
	if (abs(proj_top - proj_bottom) < cCapsuleProjectionSlop * len)
	{
		outVertices.push_back(inCenterOfMassTransform * support_top);
		outVertices.push_back(inCenterOfMassTransform * support_bottom);
	}
}

MassProperties TaperedCapsuleShape::GetMassProperties() const
{
	AABox box = GetInertiaApproximation();

	MassProperties p;
	p.SetMassAndInertiaOfSolidBox(box.GetSize(), GetDensity());
	return p;
}

Vec3 TaperedCapsuleShape::GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const 
{ 
	JPH_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID"); 

	// See: TaperedCapsuleShape.gliffy
	// We need to calculate ty and by in order to see if the position is on the top or bottom sphere
	// sin(alpha) = by / br = ty / tr
	// => by = sin(alpha) * br, ty = sin(alpha) * tr

	if (inLocalSurfacePosition.GetY() > mTopCenter + mSinAlpha * mTopRadius)
		return (inLocalSurfacePosition - Vec3(C0, mTopCenter, C0)).Normalized();
	else if (inLocalSurfacePosition.GetY() < mBottomCenter + mSinAlpha * mBottomRadius)
		return (inLocalSurfacePosition - Vec3(C0, mBottomCenter, C0)).Normalized();
	else
	{
		// Get perpendicular vector to the surface in the xz plane
		Vec3 perpendicular = Vec3(inLocalSurfacePosition.GetX(), C0, inLocalSurfacePosition.GetZ()).NormalizedOr(Vec3::sAxisX());

		// We know that the perpendicular has length 1 and that it needs a y component where tan(alpha) = y / 1 in order to align it to the surface
		perpendicular.SetY(mTanAlpha);
		return perpendicular.Normalized();
	}
}

AABox TaperedCapsuleShape::GetLocalBounds() const
{
	decimal max_radius = max(mTopRadius, mBottomRadius);
	return AABox(Vec3(-max_radius, mBottomCenter - mBottomRadius, -max_radius), Vec3(max_radius, mTopCenter + mTopRadius, max_radius));
}

AABox TaperedCapsuleShape::GetWorldSpaceBounds(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale) const
{ 
	JPH_ASSERT(IsValidScale(inScale));

	Vec3 abs_scale = inScale.Abs();
	decimal scale_xz = abs_scale.GetX();
	decimal scale_y = inScale.GetY(); // The sign of y is important as it flips the tapered capsule
	Vec3 bottom_extent = Vec3::sReplicate(scale_xz * mBottomRadius);
	Vec3 bottom_center = inCenterOfMassTransform * Vec3(C0, scale_y * mBottomCenter, C0);
	Vec3 top_extent = Vec3::sReplicate(scale_xz * mTopRadius);
	Vec3 top_center = inCenterOfMassTransform * Vec3(C0, scale_y * mTopCenter, C0);
	Vec3 p1 = Vec3::sMin(top_center - top_extent, bottom_center - bottom_extent);
	Vec3 p2 = Vec3::sMax(top_center + top_extent, bottom_center + bottom_extent);
	return AABox(p1, p2);
}

#ifdef JPH_DEBUG_RENDERER
void TaperedCapsuleShape::Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const
{
	if (mGeometry == nullptr)
	{
		SupportBuffer buffer;
		const Support *support = GetSupportFunction(ESupportMode::IncludeConvexRadius, buffer, Vec3::sReplicate(C1));
		mGeometry = inRenderer->CreateTriangleGeometryForConvex([support](Vec3Arg inDirection) { return support->GetSupport(inDirection); });
	}

	// Preserve flip along y axis but make sure we're not inside out
	Vec3 scale = ScaleHelpers::IsInsideOut(inScale)? Vec3(-C1, C1, C1) * inScale : inScale;
	RMat44 world_transform = inCenterOfMassTransform * Mat44::sScale(scale);

	AABox bounds = Shape::GetWorldSpaceBounds(inCenterOfMassTransform, inScale);

	decimal lod_scale_sq = Square(max(mTopRadius, mBottomRadius));

	Color color = inUseMaterialColors? GetMaterial()->GetDebugColor() : inColor;

	DebugRenderer::EDrawMode draw_mode = inDrawWireframe? DebugRenderer::EDrawMode::Wireframe : DebugRenderer::EDrawMode::Solid;

	inRenderer->DrawGeometry(world_transform, bounds, lod_scale_sq, color, mGeometry, DebugRenderer::ECullMode::CullBackFace, DebugRenderer::ECastShadow::On, draw_mode);
}
#endif // JPH_DEBUG_RENDERER

AABox TaperedCapsuleShape::GetInertiaApproximation() const
{
	// TODO: For now the mass and inertia is that of a box
	decimal avg_radius = C0P5 * (mTopRadius + mBottomRadius);
	return AABox(Vec3(-avg_radius, mBottomCenter - mBottomRadius, -avg_radius), Vec3(avg_radius, mTopCenter + mTopRadius, avg_radius));
}

void TaperedCapsuleShape::TransformShape(Mat44Arg inCenterOfMassTransform, TransformedShapeCollector &ioCollector) const
{
	Vec3 scale;
	Mat44 transform = inCenterOfMassTransform.Decompose(scale);
	TransformedShape ts(RVec3(transform.GetTranslation()), transform.GetRotation().GetQuaternion(), this, BodyID(), SubShapeIDCreator());
	ts.SetShapeScale(scale.GetSign() * ScaleHelpers::MakeUniformScale(scale.Abs()));
	ioCollector.AddHit(ts);
}

void TaperedCapsuleShape::SaveBinaryState(StreamOut &inStream) const
{
	ConvexShape::SaveBinaryState(inStream);

	inStream.Write(mCenterOfMass);
	inStream.Write(mTopRadius);
	inStream.Write(mBottomRadius);
	inStream.Write(mTopCenter);
	inStream.Write(mBottomCenter);
	inStream.Write(mConvexRadius);
	inStream.Write(mSinAlpha);
	inStream.Write(mTanAlpha);
}

void TaperedCapsuleShape::RestoreBinaryState(StreamIn &inStream)
{
	ConvexShape::RestoreBinaryState(inStream);

	inStream.Read(mCenterOfMass);
	inStream.Read(mTopRadius);
	inStream.Read(mBottomRadius);
	inStream.Read(mTopCenter);
	inStream.Read(mBottomCenter);
	inStream.Read(mConvexRadius);
	inStream.Read(mSinAlpha);
	inStream.Read(mTanAlpha);
}

bool TaperedCapsuleShape::IsValidScale(Vec3Arg inScale) const
{
	return ConvexShape::IsValidScale(inScale) && ScaleHelpers::IsUniformScale(inScale.Abs());
}

void TaperedCapsuleShape::sRegister()
{
	ShapeFunctions &f = ShapeFunctions::sGet(EShapeSubType::TaperedCapsule);
	f.mConstruct = []() -> Shape * { return new TaperedCapsuleShape; };
	f.mColor = Color::sGreen;
}

JPH_NAMESPACE_END
