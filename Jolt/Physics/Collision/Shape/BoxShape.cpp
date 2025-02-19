// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/ScaleHelpers.h>
#include <Jolt/Physics/Collision/Shape/GetTrianglesContext.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/Geometry/RayAABox.h>
#include <Jolt/ObjectStream/TypeDeclarations.h>
#include <Jolt/Core/StreamIn.h>
#include <Jolt/Core/StreamOut.h>
#ifdef JPH_DEBUG_RENDERER
	#include <Jolt/Renderer/DebugRenderer.h>
#endif // JPH_DEBUG_RENDERER

JPH_NAMESPACE_BEGIN

JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(BoxShapeSettings)
{
	JPH_ADD_BASE_CLASS(BoxShapeSettings, ConvexShapeSettings)

	JPH_ADD_ATTRIBUTE(BoxShapeSettings, mHalfExtent)
	JPH_ADD_ATTRIBUTE(BoxShapeSettings, mConvexRadius)
}

static const Vec3 sUnitBoxTriangles[] = {
	Vec3(-C1, C1, -C1),	Vec3(-C1, C1, C1),		Vec3(C1, C1, C1),
	Vec3(-C1, C1, -C1),	Vec3(C1, C1, C1),		Vec3(C1, C1, -C1),
	Vec3(-C1, -C1, -C1),	Vec3(C1, -C1, -C1),	Vec3(C1, -C1, C1),
	Vec3(-C1, -C1, -C1),	Vec3(C1, -C1, C1),		Vec3(-C1, -C1, C1),
	Vec3(-C1, C1, -C1),	Vec3(-C1, -C1, -C1),	Vec3(-C1, -C1, C1),
	Vec3(-C1, C1, -C1),	Vec3(-C1, -C1, C1),	Vec3(-C1, C1, C1),
	Vec3(C1, C1, C1),		Vec3(C1, -C1, C1),		Vec3(C1, -C1, -C1),
	Vec3(C1, C1, C1),		Vec3(C1, -C1, -C1),	Vec3(C1, C1, -C1),
	Vec3(-C1, C1, C1),		Vec3(-C1, -C1, C1),	Vec3(C1, -C1, C1),
	Vec3(-C1, C1, C1),		Vec3(C1, -C1, C1),		Vec3(C1, C1, C1),
	Vec3(-C1, C1, -C1),	Vec3(C1, C1, -C1),		Vec3(C1, -C1, -C1),
	Vec3(-C1, C1, -C1),	Vec3(C1, -C1, -C1),	Vec3(-C1, -C1, -C1)
};

ShapeSettings::ShapeResult BoxShapeSettings::Create() const
{ 
	if (mCachedResult.IsEmpty())
		Ref<Shape> shape = new BoxShape(*this, mCachedResult); 
	return mCachedResult;
}

BoxShape::BoxShape(const BoxShapeSettings &inSettings, ShapeResult &outResult) : 
	ConvexShape(EShapeSubType::Box, inSettings, outResult), 
	mHalfExtent(inSettings.mHalfExtent), 
	mConvexRadius(inSettings.mConvexRadius) 
{ 
	// Check convex radius
	if (inSettings.mConvexRadius < C0
		|| inSettings.mHalfExtent.ReduceMin() <= inSettings.mConvexRadius)
	{
		outResult.SetError("Invalid convex radius");
		return;
	}

	// Result is valid
	outResult.Set(this);
}

class BoxShape::Box final : public Support
{
public:
					Box(const AABox &inBox, decimal inConvexRadius) : 
		mBox(inBox),
		mConvexRadius(inConvexRadius)
	{ 
		static_assert(sizeof(Box) <= sizeof(SupportBuffer), "Buffer size too small"); 
		JPH_ASSERT(IsAligned(this, alignof(Box)));
	}

	virtual Vec3	GetSupport(Vec3Arg inDirection) const override
	{ 
		return mBox.GetSupport(inDirection); 
	}

	virtual decimal	GetConvexRadius() const override
	{
		return mConvexRadius;
	}

private:
	AABox			mBox;
	decimal			mConvexRadius;
};

const ConvexShape::Support *BoxShape::GetSupportFunction(ESupportMode inMode, SupportBuffer &inBuffer, Vec3Arg inScale) const
{
	// Scale our half extents
	Vec3 scaled_half_extent = inScale.Abs() * mHalfExtent;

	switch (inMode)
	{
	case ESupportMode::IncludeConvexRadius:
		{
			// Make box out of our half extents
			AABox box = AABox(-scaled_half_extent, scaled_half_extent);
			JPH_ASSERT(box.IsValid());
			return new (&inBuffer) Box(box, C0);
		}

	case ESupportMode::ExcludeConvexRadius:
		{
			// Reduce the box by our convex radius
			decimal convex_radius = ScaleHelpers::ScaleConvexRadius(mConvexRadius, inScale);
			Vec3 convex_radius3 = Vec3::sReplicate(convex_radius);
			Vec3 reduced_half_extent = scaled_half_extent - convex_radius3;
			AABox box = AABox(-reduced_half_extent, reduced_half_extent);
			JPH_ASSERT(box.IsValid());
			return new (&inBuffer) Box(box, convex_radius);
		}
	}

	JPH_ASSERT(false);
	return nullptr;
}

void BoxShape::GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const 
{ 
	JPH_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID");

	Vec3 scaled_half_extent = inScale.Abs() * mHalfExtent;
	AABox box(-scaled_half_extent, scaled_half_extent);
	box.GetSupportingFace(inDirection, outVertices); 

	// Transform to world space
	for (Vec3 &v : outVertices)
		v = inCenterOfMassTransform * v;
}

MassProperties BoxShape::GetMassProperties() const
{
	MassProperties p;
	p.SetMassAndInertiaOfSolidBox(C2 * mHalfExtent, GetDensity());
	return p;
}

Vec3 BoxShape::GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const 
{ 
	JPH_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID"); 

	// Get component that is closest to the surface of the box
	int index = (inLocalSurfacePosition.Abs() - mHalfExtent).Abs().GetLowestComponentIndex();

	// Calculate normal
	Vec3 normal = Vec3::sZero();
	normal.SetComponent(index, inLocalSurfacePosition[index] > C0? C1 : -C1);
	return normal;
}

#ifdef JPH_DEBUG_RENDERER
void BoxShape::Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const
{
	DebugRenderer::EDrawMode draw_mode = inDrawWireframe? DebugRenderer::EDrawMode::Wireframe : DebugRenderer::EDrawMode::Solid;
	inRenderer->DrawBox(inCenterOfMassTransform * Mat44::sScale(inScale.Abs()), GetLocalBounds(), inUseMaterialColors? GetMaterial()->GetDebugColor() : inColor, DebugRenderer::ECastShadow::On, draw_mode);
}
#endif // JPH_DEBUG_RENDERER

bool BoxShape::CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const
{
	// Test hit against box
	decimal fraction = max(RayAABox(inRay.mOrigin, RayInvDirection(inRay.mDirection), -mHalfExtent, mHalfExtent), C0);
	if (fraction < ioHit.mFraction)
	{
		ioHit.mFraction = fraction;
		ioHit.mSubShapeID2 = inSubShapeIDCreator.GetID();
		return true;
	}
	return false;
}

void BoxShape::CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	// Test shape filter
	if (!inShapeFilter.ShouldCollide(inSubShapeIDCreator.GetID()))
		return;

	decimal min_fraction, max_fraction;
	RayAABox(inRay.mOrigin, RayInvDirection(inRay.mDirection), -mHalfExtent, mHalfExtent, min_fraction, max_fraction);
	if (min_fraction <= max_fraction // Ray should intersect
		&& max_fraction >= C0 // End of ray should be inside box
		&& min_fraction < ioCollector.GetEarlyOutFraction()) // Start of ray should be before early out fraction
	{
		// Better hit than the current hit
		RayCastResult hit;
		hit.mBodyID = TransformedShape::sGetBodyID(ioCollector.GetContext());
		hit.mSubShapeID2 = inSubShapeIDCreator.GetID();

		// Check front side
		if (inRayCastSettings.mTreatConvexAsSolid || min_fraction > C0)
		{
			hit.mFraction = max(C0, min_fraction);
			ioCollector.AddHit(hit);
		}

		// Check back side hit
		if (inRayCastSettings.mBackFaceMode == EBackFaceMode::CollideWithBackFaces 
			&& max_fraction < ioCollector.GetEarlyOutFraction())
		{
			hit.mFraction = max_fraction;
			ioCollector.AddHit(hit);
		}
	}
}

void BoxShape::CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	// Test shape filter
	if (!inShapeFilter.ShouldCollide(inSubShapeIDCreator.GetID()))
		return;

	if (Vec3::sLessOrEqual(inPoint.Abs(), mHalfExtent).TestAllXYZTrue())
		ioCollector.AddHit({ TransformedShape::sGetBodyID(ioCollector.GetContext()), inSubShapeIDCreator.GetID() });
}

void BoxShape::GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const
{
	new (&ioContext) GetTrianglesContextVertexList(inPositionCOM, inRotation, inScale, Mat44::sScale(mHalfExtent), sUnitBoxTriangles, sizeof(sUnitBoxTriangles) / sizeof(Vec3), GetMaterial());
}

int BoxShape::GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials) const
{
	return ((GetTrianglesContextVertexList &)ioContext).GetTrianglesNext(inMaxTrianglesRequested, outTriangleVertices, outMaterials);
}

void BoxShape::SaveBinaryState(StreamOut &inStream) const
{
	ConvexShape::SaveBinaryState(inStream);

	inStream.Write(mHalfExtent);
	inStream.Write(mConvexRadius);
}

void BoxShape::RestoreBinaryState(StreamIn &inStream)
{
	ConvexShape::RestoreBinaryState(inStream);

	inStream.Read(mHalfExtent);
	inStream.Read(mConvexRadius);
}

void BoxShape::sRegister()
{
	ShapeFunctions &f = ShapeFunctions::sGet(EShapeSubType::Box);
	f.mConstruct = []() -> Shape * { return new BoxShape; };
	f.mColor = Color::sGreen;
}

JPH_NAMESPACE_END
