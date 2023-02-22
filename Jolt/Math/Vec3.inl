// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Jolt/Math/Vec4.h>
#include <Jolt/Math/UVec4.h>
#include <Jolt/Core/HashCombine.h>

JPH_SUPPRESS_WARNINGS_STD_BEGIN
#include <random>
JPH_SUPPRESS_WARNINGS_STD_END

// Create a std::hash for Vec3
JPH_MAKE_HASHABLE(JPH::Vec3, t.GetX(), t.GetY(), t.GetZ())

JPH_NAMESPACE_BEGIN

void Vec3::CheckW() const
{ 
#ifdef JPH_FLOATING_POINT_EXCEPTIONS_ENABLED
	// Avoid asserts when both components are NaN
	JPH_ASSERT(reinterpret_cast<const uint32 *>(mF32)[2] == reinterpret_cast<const uint32 *>(mF32)[3]); 
#endif // JPH_FLOATING_POINT_EXCEPTIONS_ENABLED
} 
	
JPH_INLINE Vec3::Type Vec3::sFixW(Type inValue)
{ 
#ifdef JPH_FLOATING_POINT_EXCEPTIONS_ENABLED
	#if defined(JPH_USE_SSE)
		return _mm_shuffle_ps(inValue, inValue, _MM_SHUFFLE(2, 2, 1, 0)); 
	#elif defined(JPH_USE_NEON)
		return JPH_NEON_SHUFFLE_F32x4(inValue, inValue, 0, 1, 2, 2);
	#else
		Type value;
		value.mData[0] = inValue.mData[0];
		value.mData[1] = inValue.mData[1];
		value.mData[2] = inValue.mData[2];
		value.mData[3] = inValue.mData[2];
		return value;
	#endif
#else
	return inValue;
#endif // JPH_FLOATING_POINT_EXCEPTIONS_ENABLED
}

Vec3::Vec3(Vec4Arg inRHS) : 
	mValue(sFixW(inRHS.mValue))
{ 
}

Vec3::Vec3(const Float3 &inV)
{
	mF32[0] = inV[0];
	mF32[1] = inV[1];
	mF32[2] = inV[2];
	#ifdef JPH_FLOATING_POINT_EXCEPTIONS_ENABLED
		mF32[3] = inV[2];
	#endif
}

Vec3::Vec3(decimal inX, decimal inY, decimal inZ)
{
	mF32[0] = inX;
	mF32[1] = inY;
	mF32[2] = inZ;
	#ifdef JPH_FLOATING_POINT_EXCEPTIONS_ENABLED
		mF32[3] = inZ;
	#endif
}

template<uint32 SwizzleX, uint32 SwizzleY, uint32 SwizzleZ>
Vec3 Vec3::Swizzle() const
{
	static_assert(SwizzleX <= 3, "SwizzleX template parameter out of range");
	static_assert(SwizzleY <= 3, "SwizzleY template parameter out of range");
	static_assert(SwizzleZ <= 3, "SwizzleZ template parameter out of range");

	return Vec3(mF32[SwizzleX], mF32[SwizzleY], mF32[SwizzleZ]);
}

Vec3 Vec3::sZero()
{
	return Vec3(C0, C0, C0);
}

Vec3 Vec3::sReplicate(decimal inV)
{
	return Vec3(inV, inV, inV);
}

Vec3 Vec3::sNaN()
{
	throw std::exception("Not Implementation");
	//return sReplicate(numeric_limits<decimal>::quiet_NaN());
}

Vec3 Vec3::sLoadFloat3Unsafe(const Float3 &inV)
{
	Type v = { inV.x, inV.y, inV.z };
	return sFixW(v);
}

Vec3 Vec3::sMin(Vec3Arg inV1, Vec3Arg inV2)
{
	return Vec3(min(inV1.mF32[0], inV2.mF32[0]), 
				min(inV1.mF32[1], inV2.mF32[1]), 
				min(inV1.mF32[2], inV2.mF32[2]));
}

Vec3 Vec3::sMax(Vec3Arg inV1, Vec3Arg inV2)
{
	return Vec3(max(inV1.mF32[0], inV2.mF32[0]), 
				max(inV1.mF32[1], inV2.mF32[1]), 
				max(inV1.mF32[2], inV2.mF32[2]));
}

Vec3 Vec3::sClamp(Vec3Arg inV, Vec3Arg inMin, Vec3Arg inMax)
{
	return sMax(sMin(inV, inMax), inMin);
}

UVec4 Vec3::sEquals(Vec3Arg inV1, Vec3Arg inV2)
{
	uint32 z = inV1.mF32[2] == inV2.mF32[2]? 0xffffffffu : 0;
	return UVec4(inV1.mF32[0] == inV2.mF32[0]? 0xffffffffu : 0, 
				 inV1.mF32[1] == inV2.mF32[1]? 0xffffffffu : 0, 
				 z, 
				 z);
}

UVec4 Vec3::sLess(Vec3Arg inV1, Vec3Arg inV2)
{
	uint32 z = inV1.mF32[2] < inV2.mF32[2]? 0xffffffffu : 0;
	return UVec4(inV1.mF32[0] < inV2.mF32[0]? 0xffffffffu : 0, 
				 inV1.mF32[1] < inV2.mF32[1]? 0xffffffffu : 0, 
				 z, 
				 z);
}

UVec4 Vec3::sLessOrEqual(Vec3Arg inV1, Vec3Arg inV2)
{
	uint32 z = inV1.mF32[2] <= inV2.mF32[2]? 0xffffffffu : 0;
	return UVec4(inV1.mF32[0] <= inV2.mF32[0]? 0xffffffffu : 0, 
				 inV1.mF32[1] <= inV2.mF32[1]? 0xffffffffu : 0, 
				 z, 
				 z);
}

UVec4 Vec3::sGreater(Vec3Arg inV1, Vec3Arg inV2)
{
	uint32 z = inV1.mF32[2] > inV2.mF32[2]? 0xffffffffu : 0;
	return UVec4(inV1.mF32[0] > inV2.mF32[0]? 0xffffffffu : 0, 
				 inV1.mF32[1] > inV2.mF32[1]? 0xffffffffu : 0, 
				 z, 
				 z);
}

UVec4 Vec3::sGreaterOrEqual(Vec3Arg inV1, Vec3Arg inV2)
{
	uint32 z = inV1.mF32[2] >= inV2.mF32[2]? 0xffffffffu : 0;
	return UVec4(inV1.mF32[0] >= inV2.mF32[0]? 0xffffffffu : 0, 
				 inV1.mF32[1] >= inV2.mF32[1]? 0xffffffffu : 0, 
				 z, 
				 z);
}

Vec3 Vec3::sFusedMultiplyAdd(Vec3Arg inMul1, Vec3Arg inMul2, Vec3Arg inAdd)
{
	return Vec3(inMul1.mF32[0] * inMul2.mF32[0] + inAdd.mF32[0],
				inMul1.mF32[1] * inMul2.mF32[1] + inAdd.mF32[1],
				inMul1.mF32[2] * inMul2.mF32[2] + inAdd.mF32[2]);
}

Vec3 Vec3::sSelect(Vec3Arg inV1, Vec3Arg inV2, UVec4Arg inControl)
{
	Vec3 result;
	for (int i = 0; i < 3; i++)
		result.mF32[i] = inControl.mU32[i] ? inV2.mF32[i] : inV1.mF32[i];
#ifdef JPH_FLOATING_POINT_EXCEPTIONS_ENABLED
	result.mF32[3] = result.mF32[2];
#endif // JPH_FLOATING_POINT_EXCEPTIONS_ENABLED
	return result;
}

Vec3 Vec3::sOr(Vec3Arg inV1, Vec3Arg inV2)
{
	return Vec3(UVec4::sOr(inV1.ReinterpretAsInt(), inV2.ReinterpretAsInt()).ReinterpretAsFloat());
}

Vec3 Vec3::sXor(Vec3Arg inV1, Vec3Arg inV2)
{
	return Vec3(UVec4::sXor(inV1.ReinterpretAsInt(), inV2.ReinterpretAsInt()).ReinterpretAsFloat());
}

Vec3 Vec3::sAnd(Vec3Arg inV1, Vec3Arg inV2)
{
	return Vec3(UVec4::sAnd(inV1.ReinterpretAsInt(), inV2.ReinterpretAsInt()).ReinterpretAsFloat());
}

Vec3 Vec3::sUnitSpherical(decimal inTheta, decimal inPhi)
{
	Vec4 s, c;
	Vec4(inTheta, inPhi, C0, C0).SinCos(s, c);
	return Vec3(s.GetX() * c.GetY(), s.GetX() * s.GetY(), c.GetX());
}

template <class Random>
Vec3 Vec3::sRandom(Random &inRandom)
{
	std::uniform_real_distribution<decimal> zero_to_one(0.0f, 1.0f);
	decimal theta = JPH_PI * zero_to_one(inRandom);
	decimal phi = 2.0f * JPH_PI * zero_to_one(inRandom);
	return sUnitSpherical(theta, phi);
}

bool Vec3::operator == (Vec3Arg inV2) const 
{ 
	return sEquals(*this, inV2).TestAllXYZTrue();
}

bool Vec3::IsClose(Vec3Arg inV2, decimal inMaxDistSq) const
{
	return (inV2 - *this).LengthSq() <= inMaxDistSq;
}

bool Vec3::IsNearZero(decimal inMaxDistSq) const
{
	return LengthSq() <= inMaxDistSq;
}

Vec3 Vec3::operator * (Vec3Arg inV2) const
{
	return Vec3(mF32[0] * inV2.mF32[0], mF32[1] * inV2.mF32[1], mF32[2] * inV2.mF32[2]);
}

Vec3 Vec3::operator * (decimal inV2) const
{
	return Vec3(mF32[0] * inV2, mF32[1] * inV2, mF32[2] * inV2);
}

Vec3 operator * (decimal inV1, Vec3Arg inV2)
{
	return Vec3(inV1 * inV2.mF32[0], inV1 * inV2.mF32[1], inV1 * inV2.mF32[2]);
}

Vec3 Vec3::operator / (decimal inV2) const
{
	return Vec3(mF32[0] / inV2, mF32[1] / inV2, mF32[2] / inV2);
}

Vec3 &Vec3::operator *= (decimal inV2)
{
	for (int i = 0; i < 3; ++i)
		mF32[i] *= inV2;
	#ifdef JPH_FLOATING_POINT_EXCEPTIONS_ENABLED
		mF32[3] = mF32[2];
	#endif
	return *this;
}

Vec3 &Vec3::operator *= (Vec3Arg inV2)
{
	for (int i = 0; i < 3; ++i)
		mF32[i] *= inV2.mF32[i];
	#ifdef JPH_FLOATING_POINT_EXCEPTIONS_ENABLED
		mF32[3] = mF32[2];
	#endif
	return *this;
}

Vec3 &Vec3::operator /= (decimal inV2)
{
	for (int i = 0; i < 3; ++i)
		mF32[i] /= inV2;
	#ifdef JPH_FLOATING_POINT_EXCEPTIONS_ENABLED
		mF32[3] = mF32[2];
	#endif
	return *this;
}

Vec3 Vec3::operator + (Vec3Arg inV2) const
{
	return Vec3(mF32[0] + inV2.mF32[0], mF32[1] + inV2.mF32[1], mF32[2] + inV2.mF32[2]);
}

Vec3 &Vec3::operator += (Vec3Arg inV2)
{
	for (int i = 0; i < 3; ++i)
		mF32[i] += inV2.mF32[i];
	#ifdef JPH_FLOATING_POINT_EXCEPTIONS_ENABLED
		mF32[3] = mF32[2];
	#endif
	return *this;
}

Vec3 Vec3::operator - () const
{
	return Vec3(-mF32[0], -mF32[1], -mF32[2]);
}

Vec3 Vec3::operator - (Vec3Arg inV2) const
{
	return Vec3(mF32[0] - inV2.mF32[0], mF32[1] - inV2.mF32[1], mF32[2] - inV2.mF32[2]);
}

Vec3 &Vec3::operator -= (Vec3Arg inV2)
{
	for (int i = 0; i < 3; ++i)
		mF32[i] -= inV2.mF32[i];
	#ifdef JPH_FLOATING_POINT_EXCEPTIONS_ENABLED
		mF32[3] = mF32[2];
	#endif
	return *this;
}

Vec3 Vec3::operator / (Vec3Arg inV2) const
{
	inV2.CheckW(); // Check W equals Z to avoid div by zero
	return Vec3(mF32[0] / inV2.mF32[0], mF32[1] / inV2.mF32[1], mF32[2] / inV2.mF32[2]);
}

Vec4 Vec3::SplatX() const
{
	return Vec4(mF32[0], mF32[0], mF32[0], mF32[0]);
}

Vec4 Vec3::SplatY() const
{
	return Vec4(mF32[1], mF32[1], mF32[1], mF32[1]);
}

Vec4 Vec3::SplatZ() const
{
	return Vec4(mF32[2], mF32[2], mF32[2], mF32[2]);
}

int Vec3::GetLowestComponentIndex() const
{
	return GetX() < GetY() ? (GetZ() < GetX() ? 2 : 0) : (GetZ() < GetY() ? 2 : 1);
}

int Vec3::GetHighestComponentIndex() const
{
	return GetX() > GetY() ? (GetZ() > GetX() ? 2 : 0) : (GetZ() > GetY() ? 2 : 1);
}

Vec3 Vec3::Abs() const
{
	return Vec3(abs(mF32[0]), abs(mF32[1]), abs(mF32[2]));
}

Vec3 Vec3::Reciprocal() const
{
	return sReplicate(C1) / mValue;
}

Vec3 Vec3::Cross(Vec3Arg inV2) const
{
	return Vec3(mF32[1] * inV2.mF32[2] - mF32[2] * inV2.mF32[1],
				mF32[2] * inV2.mF32[0] - mF32[0] * inV2.mF32[2],
				mF32[0] * inV2.mF32[1] - mF32[1] * inV2.mF32[0]);
}

Vec3 Vec3::DotV(Vec3Arg inV2) const
{
	decimal dot = C0;
	for (int i = 0; i < 3; i++)
		dot += mF32[i] * inV2.mF32[i];
	return Vec3::sReplicate(dot);
}

Vec4 Vec3::DotV4(Vec3Arg inV2) const
{
	decimal dot = C0;
	for (int i = 0; i < 3; i++)
		dot += mF32[i] * inV2.mF32[i];
	return Vec4::sReplicate(dot);
}

decimal Vec3::Dot(Vec3Arg inV2) const
{
	decimal dot = C0;
	for (int i = 0; i < 3; i++)
		dot += mF32[i] * inV2.mF32[i];
	return dot;
}

decimal Vec3::LengthSq() const
{
	decimal len_sq = C0;
	for (int i = 0; i < 3; i++)
		len_sq += mF32[i] * mF32[i];
	return len_sq;
}

decimal_raw Vec3::LengthSqRaw() const
{
	fmedi_t len_sq = 0;
	for (int i = 0; i < 3; i++)
		len_sq += static_cast<fmedi_t>(mF32[i].raw_value()) * mF32[i].raw_value();
	return len_sq;
}

decimal Vec3::Length() const
{
	fmedi_t len_sq = 0;
	for (int i = 0; i < 3; i++)
		len_sq += static_cast<fmedi_t>(mF32[i].raw_value()) * mF32[i].raw_value();
	return sqrt(LengthSq());
}

Vec3 Vec3::Sqrt() const
{
	return Vec3(sqrt(mF32[0]), sqrt(mF32[1]), sqrt(mF32[2]));
}

Vec3 Vec3::Normalized() const
{
	return *this / Length();
}

Vec3 Vec3::NormalizedOr(Vec3Arg inZeroValue) const
{
	decimal len_sq = LengthSq();
	if (len_sq == C0)
		return inZeroValue;
	else
		return *this / sqrt(len_sq);
}

bool Vec3::IsNormalized(decimal inTolerance) const 
{ 
	return abs(LengthSq() - C1) <= inTolerance; 
}

bool Vec3::IsNaN() const
{
	return isnan(mF32[0]) || isnan(mF32[1]) || isnan(mF32[2]);
}

void Vec3::StoreFloat3(Float3 *outV) const
{
	outV->x = mF32[0];
	outV->y = mF32[1];
	outV->z = mF32[2];
}

UVec4 Vec3::ToInt() const
{
	return UVec4(uint32(mF32[0]), uint32(mF32[1]), uint32(mF32[2]), uint32(mF32[3]));
}

UVec4 Vec3::ReinterpretAsInt() const
{
	return *reinterpret_cast<const UVec4 *>(this);
}

decimal Vec3::ReduceMin() const
{
	Vec3 v = sMin(mValue, Swizzle<SWIZZLE_Y, SWIZZLE_UNUSED, SWIZZLE_Z>());
	v = sMin(v, v.Swizzle<SWIZZLE_Z, SWIZZLE_UNUSED, SWIZZLE_UNUSED>());
	return v.GetX();
}

decimal Vec3::ReduceMax() const
{
	Vec3 v = sMax(mValue, Swizzle<SWIZZLE_Y, SWIZZLE_UNUSED, SWIZZLE_Z>());
	v = sMax(v, v.Swizzle<SWIZZLE_Z, SWIZZLE_UNUSED, SWIZZLE_UNUSED>());
	return v.GetX();
}

Vec3 Vec3::GetNormalizedPerpendicular() const
{
	if (abs(mF32[0]) > abs(mF32[1]))
	{
		decimal len = sqrt(mF32[0] * mF32[0] + mF32[2] * mF32[2]);
		return Vec3(mF32[2], C0, -mF32[0]) / len;
	}
	else
	{
		decimal len = sqrt(mF32[1] * mF32[1] + mF32[2] * mF32[2]);
		return Vec3(C0, mF32[2], -mF32[1]) / len;
	}
}

Vec3 Vec3::GetSign() const
{
	return Vec3(signbit(mF32[0])? -C1 : C1, 
				signbit(mF32[1])? -C1 : C1, 
				signbit(mF32[2])? -C1 : C1);
}

JPH_NAMESPACE_END
