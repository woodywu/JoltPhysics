// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/ScaleHelpers.h>
#include <Jolt/Physics/Collision/Shape/GetTrianglesContext.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/Geometry/RayCapsule.h>
#include <Jolt/ObjectStream/TypeDeclarations.h>
#include <Jolt/Core/StreamIn.h>
#include <Jolt/Core/StreamOut.h>
#ifdef JPH_DEBUG_RENDERER
	#include <Jolt/Renderer/DebugRenderer.h>
#endif // JPH_DEBUG_RENDERER

JPH_NAMESPACE_BEGIN

JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(CapsuleShapeSettings)
{
	JPH_ADD_BASE_CLASS(CapsuleShapeSettings, ConvexShapeSettings)

	JPH_ADD_ATTRIBUTE(CapsuleShapeSettings, mRadius)
	JPH_ADD_ATTRIBUTE(CapsuleShapeSettings, mHalfHeightOfCylinder)
}

static const int cCapsuleDetailLevel = 2;

static const std::vector<Vec3> sCapsuleTopTriangles = []() { 
	std::vector<Vec3> verts;	
	GetTrianglesContextVertexList::sCreateHalfUnitSphereTop(verts, cCapsuleDetailLevel);
	return verts;
}();

static const std::vector<Vec3> sCapsuleMiddleTriangles = []() { 
	std::vector<Vec3> verts;
	GetTrianglesContextVertexList::sCreateUnitOpenCylinder(verts, cCapsuleDetailLevel);
	return verts;
}();

static const std::vector<Vec3> sCapsuleBottomTriangles = []() { 
	std::vector<Vec3> verts;	
	GetTrianglesContextVertexList::sCreateHalfUnitSphereBottom(verts, cCapsuleDetailLevel);
	return verts;
}();

ShapeSettings::ShapeResult CapsuleShapeSettings::Create() const
{ 
	if (mCachedResult.IsEmpty())
	{
		Ref<Shape> shape;
		if (IsValid() && IsSphere())
		{
			// If the capsule has no height, use a sphere instead
			shape = new SphereShape(mRadius, mMaterial);
			mCachedResult.Set(shape);
		}
		else
			shape = new CapsuleShape(*this, mCachedResult); 
	}
	return mCachedResult;
}

CapsuleShape::CapsuleShape(const CapsuleShapeSettings &inSettings, ShapeResult &outResult) : 
	ConvexShape(EShapeSubType::Capsule, inSettings, outResult), 
	mRadius(inSettings.mRadius), 
	mHalfHeightOfCylinder(inSettings.mHalfHeightOfCylinder) 
{ 
	if (inSettings.mHalfHeightOfCylinder <= C0)
	{
		outResult.SetError("Invalid height");
		return;
	}
	
	if (inSettings.mRadius <= C0)
	{
		outResult.SetError("Invalid radius");
		return;
	}

	outResult.Set(this);
}

class CapsuleShape::CapsuleNoConvex final : public Support
{
public:
					CapsuleNoConvex(Vec3Arg inHalfHeightOfCylinder, decimal inConvexRadius) : 
		mHalfHeightOfCylinder(inHalfHeightOfCylinder),
		mConvexRadius(inConvexRadius)
	{ 
		static_assert(sizeof(CapsuleNoConvex) <= sizeof(SupportBuffer), "Buffer size too small"); 
		JPH_ASSERT(IsAligned(this, alignof(CapsuleNoConvex)));
	}

	virtual Vec3	GetSupport(Vec3Arg inDirection) const override
	{ 
		if (inDirection.GetY() > C0)
			return mHalfHeightOfCylinder;
		else
			return -mHalfHeightOfCylinder;
	}

	virtual decimal	GetConvexRadius() const override
	{ 
		return mConvexRadius;
	}

private:
	Vec3			mHalfHeightOfCylinder;
	decimal			mConvexRadius;
};

class CapsuleShape::CapsuleWithConvex final : public Support
{
public:
					CapsuleWithConvex(Vec3Arg inHalfHeightOfCylinder, decimal inRadius) : 
		mHalfHeightOfCylinder(inHalfHeightOfCylinder),
		mRadius(inRadius)
	{ 
		static_assert(sizeof(CapsuleWithConvex) <= sizeof(SupportBuffer), "Buffer size too small"); 
		JPH_ASSERT(IsAligned(this, alignof(CapsuleWithConvex)));
	}

	virtual Vec3	GetSupport(Vec3Arg inDirection) const override
	{ 
		decimal len = inDirection.Length();
		Vec3 radius = len > C0? inDirection * (mRadius / len) : Vec3::sZero();

		if (inDirection.GetY() > C0)
			return radius + mHalfHeightOfCylinder;
		else
			return radius - mHalfHeightOfCylinder;
	}

	virtual decimal	GetConvexRadius() const override
	{ 
		return C0;
	}

private:
	Vec3			mHalfHeightOfCylinder;
	decimal			mRadius;
};

const ConvexShape::Support *CapsuleShape::GetSupportFunction(ESupportMode inMode, SupportBuffer &inBuffer, Vec3Arg inScale) const
{
	JPH_ASSERT(IsValidScale(inScale));

	// Get scaled capsule
	Vec3 abs_scale = inScale.Abs();
	decimal scale = abs_scale.GetX();
	Vec3 scaled_half_height_of_cylinder = Vec3(C0, scale * mHalfHeightOfCylinder, C0);
	decimal scaled_radius = scale * mRadius;

	switch (inMode)
	{
	case ESupportMode::IncludeConvexRadius:
		return new (&inBuffer) CapsuleWithConvex(scaled_half_height_of_cylinder, scaled_radius);

	case ESupportMode::ExcludeConvexRadius:
		return new (&inBuffer) CapsuleNoConvex(scaled_half_height_of_cylinder, scaled_radius);
	}

	JPH_ASSERT(false);
	return nullptr;
}

void CapsuleShape::GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const
{	
	JPH_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID");
	JPH_ASSERT(IsValidScale(inScale));

	// Get direction in horizontal plane
	Vec3 direction = inDirection;
	direction.SetComponent(1, C0);

	// Check zero vector, in this case we're hitting from top/bottom so there's no supporting face
	decimal len = direction.Length();
	if (len == C0)
		return;

	// Get scaled capsule
	Vec3 abs_scale = inScale.Abs();
	decimal scale = abs_scale.GetX();
	Vec3 scaled_half_height_of_cylinder = Vec3(C0, scale * mHalfHeightOfCylinder, C0);
	decimal scaled_radius = scale * mRadius;

	// Get support point for top and bottom sphere in the opposite of 'direction' (including convex radius)
	Vec3 support = (scaled_radius / len) * direction;
	Vec3 support_top = scaled_half_height_of_cylinder - support;
	Vec3 support_bottom = -scaled_half_height_of_cylinder - support;

	// Get projection on inDirection
	// Note that inDirection is not normalized, so we need to divide by inDirection.Length() to get the actual projection
	// We've multiplied both sides of the if below with inDirection.Length()
	decimal proj_top = support_top.Dot(inDirection);
	decimal proj_bottom = support_bottom.Dot(inDirection);

	// If projection is roughly equal then return line, otherwise we return nothing as there's only 1 point
	if (abs(proj_top - proj_bottom) < cCapsuleProjectionSlop * inDirection.Length())
	{
		outVertices.push_back(inCenterOfMassTransform * support_top);
		outVertices.push_back(inCenterOfMassTransform * support_bottom);
	}
}

MassProperties CapsuleShape::GetMassProperties() const
{
	MassProperties p;

	decimal density = GetDensity();

	// Calculate inertia and mass according to: 
	// https://www.gamedev.net/resources/_/technical/math-and-physics/capsule-inertia-tensor-r3856
	decimal radius_sq = mRadius * mRadius;
	decimal height = C2 * mHalfHeightOfCylinder;
	decimal cylinder_mass = JPH_PI * height * radius_sq * density;
	decimal hemisphere_mass = (C2 * JPH_PI / C3) * radius_sq * mRadius * density;
    
	// From cylinder
	decimal inertia_y = radius_sq * cylinder_mass * C0P5;
	decimal inertia_x = inertia_y * C0P5 + cylinder_mass * height * height / decimal(12.0f);
	decimal inertia_z = inertia_x;

	// From hemispheres
	decimal temp0 = hemisphere_mass * C2 * radius_sq / decimal(5.0f);
	inertia_y += temp0  *  C2;
	decimal temp1 = mHalfHeightOfCylinder;
	decimal temp2 = temp0 + hemisphere_mass * (temp1 * temp1 + (C3 / decimal(8.0f)) * height * mRadius);
	inertia_x += temp2 * C2;
	inertia_z += temp2 * C2;

	// Mass is cylinder + hemispheres
	p.mMass = cylinder_mass + hemisphere_mass * C2;

	// Set inertia
	p.mInertia = Mat44::sScale(Vec3(inertia_x, inertia_y, inertia_z));
	
	return p;
}

Vec3 CapsuleShape::GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const 
{ 
	JPH_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID"); 

	if (inLocalSurfacePosition.GetY() > mHalfHeightOfCylinder)
		return (inLocalSurfacePosition - Vec3(C0, mHalfHeightOfCylinder, C0)).Normalized();
	else if (inLocalSurfacePosition.GetY() < -mHalfHeightOfCylinder)
		return (inLocalSurfacePosition - Vec3(C0, -mHalfHeightOfCylinder, C0)).Normalized();
	else
		return Vec3(inLocalSurfacePosition.GetX(), C0, inLocalSurfacePosition.GetZ()).NormalizedOr(Vec3::sAxisX());
}

AABox CapsuleShape::GetLocalBounds() const
{
	Vec3 extent = Vec3::sReplicate(mRadius) + Vec3(C0, mHalfHeightOfCylinder, C0);
	return AABox(-extent, extent);
}

AABox CapsuleShape::GetWorldSpaceBounds(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale) const
{ 
	JPH_ASSERT(IsValidScale(inScale));

	Vec3 abs_scale = inScale.Abs();
	decimal scale = abs_scale.GetX();
	Vec3 extent = Vec3::sReplicate(scale * mRadius);
	Vec3 height = Vec3(C0, scale * mHalfHeightOfCylinder, C0);
	Vec3 p1 = inCenterOfMassTransform * -height;
	Vec3 p2 = inCenterOfMassTransform * height;
	return AABox(Vec3::sMin(p1, p2) - extent, Vec3::sMax(p1, p2) + extent);
}

#ifdef JPH_DEBUG_RENDERER
void CapsuleShape::Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const
{
	DebugRenderer::EDrawMode draw_mode = inDrawWireframe? DebugRenderer::EDrawMode::Wireframe : DebugRenderer::EDrawMode::Solid;
	inRenderer->DrawCapsule(inCenterOfMassTransform * Mat44::sScale(inScale.Abs().GetX()), mHalfHeightOfCylinder, mRadius, inUseMaterialColors? GetMaterial()->GetDebugColor() : inColor, DebugRenderer::ECastShadow::On, draw_mode);
}
#endif // JPH_DEBUG_RENDERER

bool CapsuleShape::CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const
{
	// Test ray against capsule
	decimal fraction = RayCapsule(inRay.mOrigin, inRay.mDirection, mHalfHeightOfCylinder, mRadius);
	if (fraction < ioHit.mFraction)
	{
		ioHit.mFraction = fraction;
		ioHit.mSubShapeID2 = inSubShapeIDCreator.GetID();
		return true;
	}
	return false;
}

void CapsuleShape::CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	// Test shape filter
	if (!inShapeFilter.ShouldCollide(inSubShapeIDCreator.GetID()))
		return;

	decimal radius_sq = Square(mRadius);

	// Get vertical distance to the top/bottom sphere centers
	decimal delta_y = abs(inPoint.GetY()) - mHalfHeightOfCylinder;

	// Get distance in horizontal plane
	decimal xz_sq = Square(inPoint.GetX()) + Square(inPoint.GetZ());

	// Check if the point is in one of the two spheres
	bool in_sphere = xz_sq + Square(delta_y) <= radius_sq;

	// Check if the point is in the cylinder in the middle
	bool in_cylinder = delta_y <= C0 && xz_sq <= radius_sq;

	if (in_sphere || in_cylinder)
		ioCollector.AddHit({ TransformedShape::sGetBodyID(ioCollector.GetContext()), inSubShapeIDCreator.GetID() });
}

void CapsuleShape::TransformShape(Mat44Arg inCenterOfMassTransform, TransformedShapeCollector &ioCollector) const
{
	Vec3 scale;
	Mat44 transform = inCenterOfMassTransform.Decompose(scale);
	TransformedShape ts(RVec3(transform.GetTranslation()), transform.GetRotation().GetQuaternion(), this, BodyID(), SubShapeIDCreator());
	ts.SetShapeScale(ScaleHelpers::MakeUniformScale(scale.Abs()));
	ioCollector.AddHit(ts);
}

void CapsuleShape::GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const
{
	JPH_ASSERT(IsValidScale(inScale));

	Vec3 abs_scale = inScale.Abs();
	decimal scale = abs_scale.GetX();

	GetTrianglesContextMultiVertexList *context = new (&ioContext) GetTrianglesContextMultiVertexList(false, GetMaterial());

	Mat44 world_matrix = Mat44::sRotationTranslation(inRotation, inPositionCOM) * Mat44::sScale(scale);

	Mat44 top_matrix = world_matrix * Mat44(Vec4(mRadius, C0, C0, C0), Vec4(C0, mRadius, C0, C0), Vec4(C0, C0, mRadius, C0), Vec4(C0, mHalfHeightOfCylinder, C0, C1));
	context->AddPart(top_matrix, sCapsuleTopTriangles.data(), sCapsuleTopTriangles.size());

	Mat44 middle_matrix = world_matrix * Mat44::sScale(Vec3(mRadius, mHalfHeightOfCylinder, mRadius));
	context->AddPart(middle_matrix, sCapsuleMiddleTriangles.data(), sCapsuleMiddleTriangles.size());

	Mat44 bottom_matrix = world_matrix * Mat44(Vec4(mRadius, C0, C0, C0), Vec4(C0, mRadius, C0, C0), Vec4(C0, C0, mRadius, C0), Vec4(C0, -mHalfHeightOfCylinder, C0, C1));
	context->AddPart(bottom_matrix, sCapsuleBottomTriangles.data(), sCapsuleBottomTriangles.size());
}

int CapsuleShape::GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials) const
{
	return ((GetTrianglesContextMultiVertexList &)ioContext).GetTrianglesNext(inMaxTrianglesRequested, outTriangleVertices, outMaterials);
}

void CapsuleShape::SaveBinaryState(StreamOut &inStream) const
{
	ConvexShape::SaveBinaryState(inStream);

	inStream.Write(mRadius);
	inStream.Write(mHalfHeightOfCylinder);
}

void CapsuleShape::RestoreBinaryState(StreamIn &inStream)
{
	ConvexShape::RestoreBinaryState(inStream);

	inStream.Read(mRadius);
	inStream.Read(mHalfHeightOfCylinder);
}

bool CapsuleShape::IsValidScale(Vec3Arg inScale) const
{
	return ConvexShape::IsValidScale(inScale) && ScaleHelpers::IsUniformScale(inScale.Abs());
}

void CapsuleShape::sRegister()
{
	ShapeFunctions &f = ShapeFunctions::sGet(EShapeSubType::Capsule);
	f.mConstruct = []() -> Shape * { return new CapsuleShape; };
	f.mColor = Color::sGreen;
}

JPH_NAMESPACE_END
