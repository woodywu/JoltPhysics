// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Jolt/Math/Vec4.h>
#include <Jolt/Math/UVec4.h>
#include <Jolt/Core/HashCombine.h>

JPH_SUPPRESS_WARNINGS_STD_BEGIN
#include <random>
JPH_SUPPRESS_WARNINGS_STD_END

// Create a std::hash for FixedVec3
JPH_MAKE_HASHABLE(JPH::FixedVec3, t.GetX(), t.GetY(), t.GetZ())

JPH_NAMESPACE_BEGIN

void FixedVec3::CheckW() const
{ 
#ifdef JPH_FLOATING_POINT_EXCEPTIONS_ENABLED
	// Avoid asserts when both components are NaN
	JPH_ASSERT(reinterpret_cast<const uint32 *>(mF32)[2] == reinterpret_cast<const uint32 *>(mF32)[3]); 
#endif // JPH_FLOATING_POINT_EXCEPTIONS_ENABLED
} 
	
JPH_INLINE FixedVec3::Type FixedVec3::sFixW(Type inValue)
{ 
#ifdef JPH_FLOATING_POINT_EXCEPTIONS_ENABLED
	Type value;
	value.mData[0] = inValue.mData[0];
	value.mData[1] = inValue.mData[1];
	value.mData[2] = inValue.mData[2];
	value.mData[3] = inValue.mData[2];
	return value;
#else
	return inValue;
#endif // JPH_FLOATING_POINT_EXCEPTIONS_ENABLED
}

FixedVec3::FixedVec3(Vec3Arg inRHS) :
	mValue(sFixW(inRHS.mValue))
{
}

FixedVec3::FixedVec3(Vec4Arg inRHS) : 
	mValue(sFixW(inRHS.mValue))
{ 
}

FixedVec3::FixedVec3(const Fixed3 &inV)
{
	mF32[0] = inV[0];
	mF32[1] = inV[1];
	mF32[2] = inV[2];
#ifdef JPH_FLOATING_POINT_EXCEPTIONS_ENABLED
	mF32[3] = inV[2];
#endif
}

FixedVec3::FixedVec3(decimal inX, decimal inY, decimal inZ)
{
	mF32[0] = inX;
	mF32[1] = inY;
	mF32[2] = inZ;
#ifdef JPH_FLOATING_POINT_EXCEPTIONS_ENABLED
	mF32[3] = inZ;
#endif
}

template<uint32 SwizzleX, uint32 SwizzleY, uint32 SwizzleZ>
FixedVec3 FixedVec3::Swizzle() const
{
	static_assert(SwizzleX <= 3, "SwizzleX template parameter out of range");
	static_assert(SwizzleY <= 3, "SwizzleY template parameter out of range");
	static_assert(SwizzleZ <= 3, "SwizzleZ template parameter out of range");

	return FixedVec3(mF32[SwizzleX], mF32[SwizzleY], mF32[SwizzleZ]);
}

FixedVec3 FixedVec3::sZero()
{
	return FixedVec3(decimal_zero, decimal_zero, decimal_zero);
}

FixedVec3 FixedVec3::sReplicate(decimal inV)
{
	return FixedVec3(inV, inV, inV);
}

FixedVec3 FixedVec3::sNaN()
{
	return sReplicate(numeric_limits<decimal>::quiet_NaN());
}

FixedVec3 FixedVec3::sLoadFixed3Unsafe(const Fixed3 &inV)
{
	Type v = { inV.x, inV.y, inV.z };
	return sFixW(v);
}

FixedVec3 FixedVec3::sMin(FixedVec3Arg inV1, FixedVec3Arg inV2)
{
	return FixedVec3(min(inV1.mF32[0], inV2.mF32[0]), 
				min(inV1.mF32[1], inV2.mF32[1]), 
				min(inV1.mF32[2], inV2.mF32[2]));
}

FixedVec3 FixedVec3::sMax(FixedVec3Arg inV1, FixedVec3Arg inV2)
{
	return FixedVec3(max(inV1.mF32[0], inV2.mF32[0]), 
				max(inV1.mF32[1], inV2.mF32[1]), 
				max(inV1.mF32[2], inV2.mF32[2]));
}

FixedVec3 FixedVec3::sClamp(FixedVec3Arg inV, FixedVec3Arg inMin, FixedVec3Arg inMax)
{
	return sMax(sMin(inV, inMax), inMin);
}

UVec4 FixedVec3::sEquals(FixedVec3Arg inV1, FixedVec3Arg inV2)
{
	uint32 z = inV1.mF32[2] == inV2.mF32[2]? 0xffffffffu : 0;
	return UVec4(inV1.mF32[0] == inV2.mF32[0]? 0xffffffffu : 0, 
				 inV1.mF32[1] == inV2.mF32[1]? 0xffffffffu : 0, 
				 z, 
				 z);
}

UVec4 FixedVec3::sLess(FixedVec3Arg inV1, FixedVec3Arg inV2)
{
	uint32 z = inV1.mF32[2] < inV2.mF32[2]? 0xffffffffu : 0;
	return UVec4(inV1.mF32[0] < inV2.mF32[0]? 0xffffffffu : 0, 
				 inV1.mF32[1] < inV2.mF32[1]? 0xffffffffu : 0, 
				 z, 
				 z);
}

UVec4 FixedVec3::sLessOrEqual(FixedVec3Arg inV1, FixedVec3Arg inV2)
{
	uint32 z = inV1.mF32[2] <= inV2.mF32[2]? 0xffffffffu : 0;
	return UVec4(inV1.mF32[0] <= inV2.mF32[0]? 0xffffffffu : 0, 
				 inV1.mF32[1] <= inV2.mF32[1]? 0xffffffffu : 0, 
				 z, 
				 z);
}

UVec4 FixedVec3::sGreater(FixedVec3Arg inV1, FixedVec3Arg inV2)
{
	uint32 z = inV1.mF32[2] > inV2.mF32[2]? 0xffffffffu : 0;
	return UVec4(inV1.mF32[0] > inV2.mF32[0]? 0xffffffffu : 0, 
				 inV1.mF32[1] > inV2.mF32[1]? 0xffffffffu : 0, 
				 z, 
				 z);
}

UVec4 FixedVec3::sGreaterOrEqual(FixedVec3Arg inV1, FixedVec3Arg inV2)
{
	uint32 z = inV1.mF32[2] >= inV2.mF32[2]? 0xffffffffu : 0;
	return UVec4(inV1.mF32[0] >= inV2.mF32[0]? 0xffffffffu : 0, 
				 inV1.mF32[1] >= inV2.mF32[1]? 0xffffffffu : 0, 
				 z, 
				 z);
}

FixedVec3 FixedVec3::sFusedMultiplyAdd(FixedVec3Arg inMul1, FixedVec3Arg inMul2, FixedVec3Arg inAdd)
{
	return FixedVec3(inMul1.mF32[0] * inMul2.mF32[0] + inAdd.mF32[0],
				inMul1.mF32[1] * inMul2.mF32[1] + inAdd.mF32[1],
				inMul1.mF32[2] * inMul2.mF32[2] + inAdd.mF32[2]);
}

FixedVec3 FixedVec3::sSelect(FixedVec3Arg inV1, FixedVec3Arg inV2, UVec4Arg inControl)
{
	FixedVec3 result;
	for (int i = 0; i < 3; i++)
		result.mF32[i] = inControl.mU32[i] ? inV2.mF32[i] : inV1.mF32[i];
#ifdef JPH_FLOATING_POINT_EXCEPTIONS_ENABLED
	result.mF32[3] = result.mF32[2];
#endif // JPH_FLOATING_POINT_EXCEPTIONS_ENABLED
	return result;
}

FixedVec3 FixedVec3::sOr(FixedVec3Arg inV1, FixedVec3Arg inV2)
{
	return FixedVec3(UVec4::sOr(inV1.ReinterpretAsInt(), inV2.ReinterpretAsInt()).ReinterpretAsFloat());
}

FixedVec3 FixedVec3::sXor(FixedVec3Arg inV1, FixedVec3Arg inV2)
{
	return FixedVec3(UVec4::sXor(inV1.ReinterpretAsInt(), inV2.ReinterpretAsInt()).ReinterpretAsFloat());
}

FixedVec3 FixedVec3::sAnd(FixedVec3Arg inV1, FixedVec3Arg inV2)
{
	return FixedVec3(UVec4::sAnd(inV1.ReinterpretAsInt(), inV2.ReinterpretAsInt()).ReinterpretAsFloat());
}

FixedVec3 FixedVec3::sUnitSpherical(float inTheta, float inPhi)
{
	Vec4 s, c;
	Vec4(inTheta, inPhi, 0, 0).SinCos(s, c);
	return FixedVec3(decimal(s.GetX() * c.GetY()), decimal(s.GetX() * s.GetY()), decimal(c.GetX()));
}

template <class Random>
FixedVec3 FixedVec3::sRandom(Random &inRandom)
{
	std::uniform_real_distribution<decimal> zero_to_one(0.0f, 1.0f);
	decimal theta = JPH_PI * zero_to_one(inRandom);
	decimal phi = 2.0f * JPH_PI * zero_to_one(inRandom);
	return sUnitSpherical(theta, phi);
}

bool FixedVec3::operator == (FixedVec3Arg inV2) const 
{ 
	return sEquals(*this, inV2).TestAllXYZTrue();
}

bool FixedVec3::IsClose(FixedVec3Arg inV2, decimal inMaxDistSq) const
{
	return (inV2 - *this).LengthSq() <= inMaxDistSq;
}

bool FixedVec3::IsNearZero(decimal inMaxDistSq) const
{
	return LengthSq() <= inMaxDistSq;
}

FixedVec3 FixedVec3::operator * (FixedVec3Arg inV2) const
{
	return FixedVec3(mF32[0] * inV2.mF32[0], mF32[1] * inV2.mF32[1], mF32[2] * inV2.mF32[2]);
}

FixedVec3 FixedVec3::operator * (decimal inV2) const
{
	return FixedVec3(mF32[0] * inV2, mF32[1] * inV2, mF32[2] * inV2);
}

FixedVec3 operator * (decimal inV1, FixedVec3Arg inV2)
{
	return FixedVec3(inV1 * inV2.mF32[0], inV1 * inV2.mF32[1], inV1 * inV2.mF32[2]);
}

FixedVec3 FixedVec3::operator / (decimal inV2) const
{
	return FixedVec3(mF32[0] / inV2, mF32[1] / inV2, mF32[2] / inV2);
}

FixedVec3 &FixedVec3::operator *= (decimal inV2)
{
	for (int i = 0; i < 3; ++i)
		mF32[i] *= inV2;
#ifdef JPH_FLOATING_POINT_EXCEPTIONS_ENABLED
		mF32[3] = mF32[2];
#endif
	return *this;
}

FixedVec3 &FixedVec3::operator *= (FixedVec3Arg inV2)
{
	for (int i = 0; i < 3; ++i)
		mF32[i] *= inV2.mF32[i];
#ifdef JPH_FLOATING_POINT_EXCEPTIONS_ENABLED
	mF32[3] = mF32[2];
#endif
	return *this;
}

FixedVec3 &FixedVec3::operator /= (decimal inV2)
{
	for (int i = 0; i < 3; ++i)
		mF32[i] /= inV2;
#ifdef JPH_FLOATING_POINT_EXCEPTIONS_ENABLED
		mF32[3] = mF32[2];
#endif
	return *this;
}

FixedVec3 FixedVec3::operator + (FixedVec3Arg inV2) const
{
	return FixedVec3(mF32[0] + inV2.mF32[0], mF32[1] + inV2.mF32[1], mF32[2] + inV2.mF32[2]);
}

FixedVec3 &FixedVec3::operator += (FixedVec3Arg inV2)
{
	for (int i = 0; i < 3; ++i)
		mF32[i] += inV2.mF32[i];
	#ifdef JPH_FLOATING_POINT_EXCEPTIONS_ENABLED
		mF32[3] = mF32[2];
	#endif
	return *this;
}

FixedVec3 FixedVec3::operator - () const
{
	return FixedVec3(-mF32[0], -mF32[1], -mF32[2]);
}

FixedVec3 FixedVec3::operator - (FixedVec3Arg inV2) const
{
	return FixedVec3(mF32[0] - inV2.mF32[0], mF32[1] - inV2.mF32[1], mF32[2] - inV2.mF32[2]);
}

FixedVec3 &FixedVec3::operator -= (FixedVec3Arg inV2)
{
	for (int i = 0; i < 3; ++i)
		mF32[i] -= inV2.mF32[i];
	#ifdef JPH_FLOATING_POINT_EXCEPTIONS_ENABLED
		mF32[3] = mF32[2];
	#endif
	return *this;
}

FixedVec3 FixedVec3::operator / (FixedVec3Arg inV2) const
{
	inV2.CheckW(); // Check W equals Z to avoid div by zero
	return FixedVec3(mF32[0] / inV2.mF32[0], mF32[1] / inV2.mF32[1], mF32[2] / inV2.mF32[2]);
}

Vec4 FixedVec3::SplatX() const
{
	return Vec4(static_cast<float>(mF32[0]),
		static_cast<float>(mF32[0]),
		static_cast<float>(mF32[0]),
		static_cast<float>(mF32[0]));
}

Vec4 FixedVec3::SplatY() const
{
	return Vec4(static_cast<float>(mF32[1]),
		static_cast<float>(mF32[1]),
		static_cast<float>(mF32[1]),
		static_cast<float>(mF32[1]));
}

Vec4 FixedVec3::SplatZ() const
{
	return Vec4(static_cast<float>(mF32[2]),
		static_cast<float>(mF32[2]),
		static_cast<float>(mF32[2]),
		static_cast<float>(mF32[2]));
}

int FixedVec3::GetLowestComponentIndex() const
{
	return GetX() < GetY() ? (GetZ() < GetX() ? 2 : 0) : (GetZ() < GetY() ? 2 : 1);
}

int FixedVec3::GetHighestComponentIndex() const
{
	return GetX() > GetY() ? (GetZ() > GetX() ? 2 : 0) : (GetZ() > GetY() ? 2 : 1);
}

FixedVec3 FixedVec3::Abs() const
{
	return FixedVec3(abs(mF32[0]), abs(mF32[1]), abs(mF32[2]));
}

FixedVec3 FixedVec3::Reciprocal() const
{
	return sReplicate(decimal_one) / mValue;
}

FixedVec3 FixedVec3::Cross(FixedVec3Arg inV2) const
{
	return FixedVec3(mF32[1] * inV2.mF32[2] - mF32[2] * inV2.mF32[1],
				mF32[2] * inV2.mF32[0] - mF32[0] * inV2.mF32[2],
				mF32[0] * inV2.mF32[1] - mF32[1] * inV2.mF32[0]);
}

FixedVec3 FixedVec3::DotV(FixedVec3Arg inV2) const
{
	decimal dot = decimal_zero;
	for (int i = 0; i < 3; i++)
		dot += mF32[i] * inV2.mF32[i];
	return FixedVec3::sReplicate(dot);
}

Vec4 FixedVec3::DotV4(FixedVec3Arg inV2) const
{
	decimal dot  = decimal_zero;
	for (int i = 0; i < 3; i++)
		dot += mF32[i] * inV2.mF32[i];
	return Vec4::sReplicate(static_cast<float>(dot));
}

decimal FixedVec3::Dot(FixedVec3Arg inV2) const
{
	decimal dot = decimal_zero;
	for (int i = 0; i < 3; i++)
		dot += mF32[i] * inV2.mF32[i];
	return dot;
}

decimal FixedVec3::LengthSq() const
{
	decimal len_sq = decimal_zero;
	for (int i = 0; i < 3; i++)
		len_sq += mF32[i] * mF32[i];
	return len_sq;
}

decimal FixedVec3::Length() const
{
	return sqrt(LengthSq());
}

FixedVec3 FixedVec3::Sqrt() const
{
	return FixedVec3(sqrt(mF32[0]), sqrt(mF32[1]), sqrt(mF32[2]));
}

FixedVec3 FixedVec3::Normalized() const
{
	return *this / Length();
}

FixedVec3 FixedVec3::NormalizedOr(FixedVec3Arg inZeroValue) const
{
	decimal len_sq = LengthSq();
	if (len_sq == decimal{0})
		return inZeroValue;
	else
		return *this / sqrt(len_sq);
}

bool FixedVec3::IsNormalized(decimal inTolerance) const
{ 
	return abs(LengthSq() - decimal_one) <= inTolerance;
}

bool FixedVec3::IsNaN() const
{
	return isnan(mF32[0]) || isnan(mF32[1]) || isnan(mF32[2]);
}

void FixedVec3::StoreFixed3(Fixed3 *outV) const
{
	outV->x = mF32[0];
	outV->y = mF32[1];
	outV->z = mF32[2];
}

UVec4 FixedVec3::ToInt() const
{
	return UVec4(uint32(mF32[0]), uint32(mF32[1]), uint32(mF32[2]), uint32(mF32[3]));
}

UVec4 FixedVec3::ReinterpretAsInt() const
{
	return *reinterpret_cast<const UVec4 *>(this);
}

decimal FixedVec3::ReduceMin() const
{
	FixedVec3 v = sMin(mValue, Swizzle<SWIZZLE_Y, SWIZZLE_UNUSED, SWIZZLE_Z>());
	v = sMin(v, v.Swizzle<SWIZZLE_Z, SWIZZLE_UNUSED, SWIZZLE_UNUSED>());
	return v.GetX();
}

decimal FixedVec3::ReduceMax() const
{
	FixedVec3 v = sMax(mValue, Swizzle<SWIZZLE_Y, SWIZZLE_UNUSED, SWIZZLE_Z>());
	v = sMax(v, v.Swizzle<SWIZZLE_Z, SWIZZLE_UNUSED, SWIZZLE_UNUSED>());
	return v.GetX();
}

FixedVec3 FixedVec3::GetNormalizedPerpendicular() const
{
	if (abs(mF32[0]) > abs(mF32[1]))
	{
		decimal len = sqrt(mF32[0] * mF32[0] + mF32[2] * mF32[2]);
		return FixedVec3(mF32[2], decimal_zero, -mF32[0]) / len;
	}
	else
	{
		decimal len = sqrt(mF32[1] * mF32[1] + mF32[2] * mF32[2]);
		return FixedVec3(decimal_zero, mF32[2], -mF32[1]) / len;
	}
}

FixedVec3 FixedVec3::GetSign() const
{
	return FixedVec3(signbit(mF32[0])? -decimal_one : decimal_one,
				signbit(mF32[1])? -decimal_one : decimal_one, 
				signbit(mF32[2])? -decimal_one : decimal_one);
}

JPH_NAMESPACE_END
