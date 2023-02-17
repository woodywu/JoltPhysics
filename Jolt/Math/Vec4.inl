// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Jolt/Math/Trigonometry.h>
#include <Jolt/Math/Vec3.h>
#include <Jolt/Math/UVec4.h>

JPH_NAMESPACE_BEGIN

// Constructor
Vec4::Vec4(Vec3Arg inRHS) : 
	mValue(inRHS.mValue) 
{ 
}

Vec4::Vec4(Vec3Arg inRHS, decimal inW)
{
	for (int i = 0; i < 3; i++)
		mF32[i] = inRHS.mF32[i];
	mF32[3] = inW;
}

Vec4::Vec4(decimal inX, decimal inY, decimal inZ, decimal inW)
{
	mF32[0] = inX;
	mF32[1] = inY;
	mF32[2] = inZ;
	mF32[3] = inW;
}

template<uint32 SwizzleX, uint32 SwizzleY, uint32 SwizzleZ, uint32 SwizzleW>
Vec4 Vec4::Swizzle() const
{
	static_assert(SwizzleX <= 3, "SwizzleX template parameter out of range");
	static_assert(SwizzleY <= 3, "SwizzleY template parameter out of range");
	static_assert(SwizzleZ <= 3, "SwizzleZ template parameter out of range");
	static_assert(SwizzleW <= 3, "SwizzleW template parameter out of range");

	return Vec4(mF32[SwizzleX], mF32[SwizzleY], mF32[SwizzleZ], mF32[SwizzleW]);
}

Vec4 Vec4::sZero()
{
	return Vec4(C0, C0, C0, C0);
}

Vec4 Vec4::sReplicate(decimal inV)
{
	return Vec4(inV, inV, inV, inV);
}

Vec4 Vec4::sNaN()
{
	throw std::exception("Not Implementation");
	//return sReplicate(numeric_limits<decimal>::quiet_NaN());
}

Vec4 Vec4::sLoadFloat4(const Float4 *inV)
{
	return Vec4(inV->x, inV->y, inV->z, inV->w);
}

Vec4 Vec4::sLoadFloat4Aligned(const Float4 *inV)
{
	return Vec4(inV->x, inV->y, inV->z, inV->w);
}

template <const int Scale>
Vec4 Vec4::sGatherFloat4(const decimal *inBase, UVec4Arg inOffsets)
{
	const uint8 *base = reinterpret_cast<const uint8 *>(inBase);
	decimal x = *reinterpret_cast<const decimal *>(base + inOffsets.GetX() * Scale);
	decimal y = *reinterpret_cast<const decimal *>(base + inOffsets.GetY() * Scale);
	decimal z = *reinterpret_cast<const decimal *>(base + inOffsets.GetZ() * Scale);
	decimal w = *reinterpret_cast<const decimal *>(base + inOffsets.GetW() * Scale);
	return Vec4(x, y, z, w);
}

Vec4 Vec4::sMin(Vec4Arg inV1, Vec4Arg inV2)
{
	return Vec4(min(inV1.mF32[0], inV2.mF32[0]), 
				min(inV1.mF32[1], inV2.mF32[1]), 
				min(inV1.mF32[2], inV2.mF32[2]), 
				min(inV1.mF32[3], inV2.mF32[3]));
}

Vec4 Vec4::sMax(Vec4Arg inV1, Vec4Arg inV2)
{
	return Vec4(max(inV1.mF32[0], inV2.mF32[0]), 
				max(inV1.mF32[1], inV2.mF32[1]), 
				max(inV1.mF32[2], inV2.mF32[2]), 
				max(inV1.mF32[3], inV2.mF32[3]));
}

UVec4 Vec4::sEquals(Vec4Arg inV1, Vec4Arg inV2)
{
	return UVec4(inV1.mF32[0] == inV2.mF32[0]? 0xffffffffu : 0, 
				 inV1.mF32[1] == inV2.mF32[1]? 0xffffffffu : 0, 
				 inV1.mF32[2] == inV2.mF32[2]? 0xffffffffu : 0, 
				 inV1.mF32[3] == inV2.mF32[3]? 0xffffffffu : 0);
}

UVec4 Vec4::sLess(Vec4Arg inV1, Vec4Arg inV2)
{
	return UVec4(inV1.mF32[0] < inV2.mF32[0]? 0xffffffffu : 0, 
				 inV1.mF32[1] < inV2.mF32[1]? 0xffffffffu : 0, 
				 inV1.mF32[2] < inV2.mF32[2]? 0xffffffffu : 0, 
				 inV1.mF32[3] < inV2.mF32[3]? 0xffffffffu : 0);
}

UVec4 Vec4::sLessOrEqual(Vec4Arg inV1, Vec4Arg inV2)
{
	return UVec4(inV1.mF32[0] <= inV2.mF32[0]? 0xffffffffu : 0, 
				 inV1.mF32[1] <= inV2.mF32[1]? 0xffffffffu : 0, 
				 inV1.mF32[2] <= inV2.mF32[2]? 0xffffffffu : 0, 
				 inV1.mF32[3] <= inV2.mF32[3]? 0xffffffffu : 0);
}

UVec4 Vec4::sGreater(Vec4Arg inV1, Vec4Arg inV2)
{
	return UVec4(inV1.mF32[0] > inV2.mF32[0]? 0xffffffffu : 0, 
				 inV1.mF32[1] > inV2.mF32[1]? 0xffffffffu : 0, 
				 inV1.mF32[2] > inV2.mF32[2]? 0xffffffffu : 0, 
				 inV1.mF32[3] > inV2.mF32[3]? 0xffffffffu : 0);
}

UVec4 Vec4::sGreaterOrEqual(Vec4Arg inV1, Vec4Arg inV2)
{
	return UVec4(inV1.mF32[0] >= inV2.mF32[0]? 0xffffffffu : 0, 
				 inV1.mF32[1] >= inV2.mF32[1]? 0xffffffffu : 0, 
				 inV1.mF32[2] >= inV2.mF32[2]? 0xffffffffu : 0, 
				 inV1.mF32[3] >= inV2.mF32[3]? 0xffffffffu : 0);
}

Vec4 Vec4::sFusedMultiplyAdd(Vec4Arg inMul1, Vec4Arg inMul2, Vec4Arg inAdd)
{
	return Vec4(inMul1.mF32[0] * inMul2.mF32[0] + inAdd.mF32[0],
				inMul1.mF32[1] * inMul2.mF32[1] + inAdd.mF32[1],
				inMul1.mF32[2] * inMul2.mF32[2] + inAdd.mF32[2],
				inMul1.mF32[3] * inMul2.mF32[3] + inAdd.mF32[3]);
}

Vec4 Vec4::sSelect(Vec4Arg inV1, Vec4Arg inV2, UVec4Arg inControl)
{
	Vec4 result;
	for (int i = 0; i < 4; i++)
		result.mF32[i] = inControl.mU32[i] ? inV2.mF32[i] : inV1.mF32[i];
	return result;
}

Vec4 Vec4::sOr(Vec4Arg inV1, Vec4Arg inV2)
{
	return UVec4::sOr(inV1.ReinterpretAsInt(), inV2.ReinterpretAsInt()).ReinterpretAsFloat();
}

Vec4 Vec4::sXor(Vec4Arg inV1, Vec4Arg inV2)
{
	return UVec4::sXor(inV1.ReinterpretAsInt(), inV2.ReinterpretAsInt()).ReinterpretAsFloat();
}

Vec4 Vec4::sAnd(Vec4Arg inV1, Vec4Arg inV2)
{
	return UVec4::sAnd(inV1.ReinterpretAsInt(), inV2.ReinterpretAsInt()).ReinterpretAsFloat();
}

void Vec4::sSort4(Vec4 &ioValue, UVec4 &ioIndex)
{
	// Pass 1, test 1st vs 3rd, 2nd vs 4th
	Vec4 v1 = ioValue.Swizzle<SWIZZLE_Z, SWIZZLE_W, SWIZZLE_X, SWIZZLE_Y>();
	UVec4 i1 = ioIndex.Swizzle<SWIZZLE_Z, SWIZZLE_W, SWIZZLE_X, SWIZZLE_Y>();
	UVec4 c1 = sLess(ioValue, v1).Swizzle<SWIZZLE_Z, SWIZZLE_W, SWIZZLE_Z, SWIZZLE_W>();
	ioValue = sSelect(ioValue, v1, c1);
	ioIndex = UVec4::sSelect(ioIndex, i1, c1);

	// Pass 2, test 1st vs 2nd, 3rd vs 4th
	Vec4 v2 = ioValue.Swizzle<SWIZZLE_Y, SWIZZLE_X, SWIZZLE_W, SWIZZLE_Z>();
	UVec4 i2 = ioIndex.Swizzle<SWIZZLE_Y, SWIZZLE_X, SWIZZLE_W, SWIZZLE_Z>();
	UVec4 c2 = sLess(ioValue, v2).Swizzle<SWIZZLE_Y, SWIZZLE_Y, SWIZZLE_W, SWIZZLE_W>();
	ioValue = sSelect(ioValue, v2, c2);
	ioIndex = UVec4::sSelect(ioIndex, i2, c2);

	// Pass 3, test 2nd vs 3rd component
	Vec4 v3 = ioValue.Swizzle<SWIZZLE_X, SWIZZLE_Z, SWIZZLE_Y, SWIZZLE_W>();
	UVec4 i3 = ioIndex.Swizzle<SWIZZLE_X, SWIZZLE_Z, SWIZZLE_Y, SWIZZLE_W>();
	UVec4 c3 = sLess(ioValue, v3).Swizzle<SWIZZLE_X, SWIZZLE_Z, SWIZZLE_Z, SWIZZLE_W>();
	ioValue = sSelect(ioValue, v3, c3);
	ioIndex = UVec4::sSelect(ioIndex, i3, c3);
}

void Vec4::sSort4Reverse(Vec4 &ioValue, UVec4 &ioIndex)
{
	// Pass 1, test 1st vs 3rd, 2nd vs 4th
	Vec4 v1 = ioValue.Swizzle<SWIZZLE_Z, SWIZZLE_W, SWIZZLE_X, SWIZZLE_Y>();
	UVec4 i1 = ioIndex.Swizzle<SWIZZLE_Z, SWIZZLE_W, SWIZZLE_X, SWIZZLE_Y>();
	UVec4 c1 = sGreater(ioValue, v1).Swizzle<SWIZZLE_Z, SWIZZLE_W, SWIZZLE_Z, SWIZZLE_W>();
	ioValue = sSelect(ioValue, v1, c1);
	ioIndex = UVec4::sSelect(ioIndex, i1, c1);

	// Pass 2, test 1st vs 2nd, 3rd vs 4th
	Vec4 v2 = ioValue.Swizzle<SWIZZLE_Y, SWIZZLE_X, SWIZZLE_W, SWIZZLE_Z>();
	UVec4 i2 = ioIndex.Swizzle<SWIZZLE_Y, SWIZZLE_X, SWIZZLE_W, SWIZZLE_Z>();
	UVec4 c2 = sGreater(ioValue, v2).Swizzle<SWIZZLE_Y, SWIZZLE_Y, SWIZZLE_W, SWIZZLE_W>();
	ioValue = sSelect(ioValue, v2, c2);
	ioIndex = UVec4::sSelect(ioIndex, i2, c2);

	// Pass 3, test 2nd vs 3rd component
	Vec4 v3 = ioValue.Swizzle<SWIZZLE_X, SWIZZLE_Z, SWIZZLE_Y, SWIZZLE_W>();
	UVec4 i3 = ioIndex.Swizzle<SWIZZLE_X, SWIZZLE_Z, SWIZZLE_Y, SWIZZLE_W>();
	UVec4 c3 = sGreater(ioValue, v3).Swizzle<SWIZZLE_X, SWIZZLE_Z, SWIZZLE_Z, SWIZZLE_W>();
	ioValue = sSelect(ioValue, v3, c3);
	ioIndex = UVec4::sSelect(ioIndex, i3, c3);
}

bool Vec4::operator == (Vec4Arg inV2) const 
{ 
	return sEquals(*this, inV2).TestAllTrue();
}

bool Vec4::IsClose(Vec4Arg inV2, decimal inMaxDistSq) const
{
	return (inV2 - *this).LengthSq() <= inMaxDistSq;
}

bool Vec4::IsNormalized(decimal inTolerance) const 
{ 
	return abs(LengthSq() - C1) <= inTolerance; 
}

bool Vec4::IsNaN() const
{
	return isnan(mF32[0]) || isnan(mF32[1]) || isnan(mF32[2]) || isnan(mF32[3]);
}

Vec4 Vec4::operator * (Vec4Arg inV2) const
{
	return Vec4(mF32[0] * inV2.mF32[0], 
				mF32[1] * inV2.mF32[1], 
				mF32[2] * inV2.mF32[2], 
				mF32[3] * inV2.mF32[3]);
}

Vec4 Vec4::operator * (decimal inV2) const
{
	return Vec4(mF32[0] * inV2, mF32[1] * inV2, mF32[2] * inV2, mF32[3] * inV2);
}

/// Multiply vector with decimal
Vec4 operator * (decimal inV1, Vec4Arg inV2)
{
	return Vec4(inV1 * inV2.mF32[0], 
				inV1 * inV2.mF32[1], 
				inV1 * inV2.mF32[2], 
				inV1 * inV2.mF32[3]);
}

Vec4 Vec4::operator / (decimal inV2) const
{
	return Vec4(mF32[0] / inV2, mF32[1] / inV2, mF32[2] / inV2, mF32[3] / inV2);
}

Vec4 &Vec4::operator *= (decimal inV2)
{
	for (int i = 0; i < 4; ++i)
		mF32[i] *= inV2;
	return *this;
}

Vec4 &Vec4::operator *= (Vec4Arg inV2)
{
	for (int i = 0; i < 4; ++i)
		mF32[i] *= inV2.mF32[i];
	return *this;
}

Vec4 &Vec4::operator /= (decimal inV2)
{
	for (int i = 0; i < 4; ++i)
		mF32[i] /= inV2;
	return *this;
}

Vec4 Vec4::operator + (Vec4Arg inV2) const
{
	return Vec4(mF32[0] + inV2.mF32[0], 
				mF32[1] + inV2.mF32[1], 
				mF32[2] + inV2.mF32[2], 
				mF32[3] + inV2.mF32[3]);
}

Vec4 &Vec4::operator += (Vec4Arg inV2)
{
	for (int i = 0; i < 4; ++i)
		mF32[i] += inV2.mF32[i];
	return *this;
}

Vec4 Vec4::operator - () const
{
	return Vec4(-mF32[0], -mF32[1], -mF32[2], -mF32[3]);
}

Vec4 Vec4::operator - (Vec4Arg inV2) const
{
	return Vec4(mF32[0] - inV2.mF32[0], 
				mF32[1] - inV2.mF32[1], 
				mF32[2] - inV2.mF32[2], 
				mF32[3] - inV2.mF32[3]);
}

Vec4 &Vec4::operator -= (Vec4Arg inV2)
{
	for (int i = 0; i < 4; ++i)
		mF32[i] -= inV2.mF32[i];
	return *this;
}

Vec4 Vec4::operator / (Vec4Arg inV2) const
{
	return Vec4(mF32[0] / inV2.mF32[0], 
				mF32[1] / inV2.mF32[1], 
				mF32[2] / inV2.mF32[2], 
				mF32[3] / inV2.mF32[3]);
}

Vec4 Vec4::SplatX() const
{
	return Vec4(mF32[0], mF32[0], mF32[0], mF32[0]);
}

Vec4 Vec4::SplatY() const
{
	return Vec4(mF32[1], mF32[1], mF32[1], mF32[1]);
}

Vec4 Vec4::SplatZ() const
{
	return Vec4(mF32[2], mF32[2], mF32[2], mF32[2]);
}

Vec4 Vec4::SplatW() const
{
	return Vec4(mF32[3], mF32[3], mF32[3], mF32[3]);
}

Vec4 Vec4::Abs() const
{
	return Vec4(abs(mF32[0]), abs(mF32[1]), abs(mF32[2]), abs(mF32[3]));
}

Vec4 Vec4::Reciprocal() const
{
	return sReplicate(C1) / mValue;
}

Vec4 Vec4::DotV(Vec4Arg inV2) const
{
	decimal dot = C0;
	for (int i = 0; i < 4; i++)
		dot += mF32[i] * inV2.mF32[i];
	return Vec4::sReplicate(dot);
}

decimal Vec4::Dot(Vec4Arg inV2) const
{
	decimal dot = C0;
	for (int i = 0; i < 4; i++)
		dot += mF32[i] * inV2.mF32[i];
	return dot;
}

decimal Vec4::LengthSq() const
{
	decimal len_sq = C0;
	for (int i = 0; i < 4; i++)
		len_sq += mF32[i] * mF32[i];
	return len_sq;
}

decimal Vec4::Length() const
{
	return sqrt(LengthSq());
}

Vec4 Vec4::Sqrt() const
{
	return Vec4(sqrt(mF32[0]), sqrt(mF32[1]), sqrt(mF32[2]), sqrt(mF32[3]));
}


Vec4 Vec4::GetSign() const
{
	return Vec4(signbit(mF32[0])? -C1 : C1, 
				signbit(mF32[1])? -C1 : C1, 
				signbit(mF32[2])? -C1 : C1, 
				signbit(mF32[3])? -C1 : C1);
}

Vec4 Vec4::Normalized() const
{
	return *this / Length();
}

void Vec4::StoreFloat4(Float4 *outV) const
{
	for (int i = 0; i < 4; ++i)
		(&outV->x)[i] = mF32[i];
}

UVec4 Vec4::ToInt() const
{
	return UVec4(uint32(mF32[0]), uint32(mF32[1]), uint32(mF32[2]), uint32(mF32[3]));
}

UVec4 Vec4::ReinterpretAsInt() const
{
	return *reinterpret_cast<const UVec4 *>(this);
}

int Vec4::GetSignBits() const
{
	return (signbit(mF32[0])? 1 : 0) | (signbit(mF32[1])? 2 : 0) | (signbit(mF32[2])? 4 : 0) | (signbit(mF32[3])? 8 : 0);
}

decimal Vec4::ReduceMin() const
{
	Vec4 v = sMin(mValue, Swizzle<SWIZZLE_Y, SWIZZLE_UNUSED, SWIZZLE_W, SWIZZLE_UNUSED>());
	v = sMin(v, v.Swizzle<SWIZZLE_Z, SWIZZLE_UNUSED, SWIZZLE_UNUSED, SWIZZLE_UNUSED>());
	return v.GetX();
}

decimal Vec4::ReduceMax() const
{
	Vec4 v = sMax(mValue, Swizzle<SWIZZLE_Y, SWIZZLE_UNUSED, SWIZZLE_W, SWIZZLE_UNUSED>());
	v = sMax(v, v.Swizzle<SWIZZLE_Z, SWIZZLE_UNUSED, SWIZZLE_UNUSED, SWIZZLE_UNUSED>());
	return v.GetX();
}

void Vec4::SinCos(Vec4 &outSin, Vec4 &outCos) const
{
	// Implementation based on sinf.c from the cephes library, combines sinf and cosf in a single function, changes octants to quadrants and vectorizes it
	// Original implementation by Stephen L. Moshier (See: http://www.moshier.net/)

	// Make argument positive and remember sign for sin only since cos is symmetric around x (highest bit of a decimal is the sign bit)
	UVec4 sin_sign = UVec4::sAnd(ReinterpretAsInt(), UVec4::sReplicate(0x80000000U));
	Vec4 x = Vec4::sXor(*this, sin_sign.ReinterpretAsFloat());

	// x / (PI / 2) rounded to nearest int gives us the quadrant closest to x
	UVec4 quadrant = (decimal(0.6366197723675814f) * x + Vec4::sReplicate(C0P5)).ToInt();

	// Make x relative to the closest quadrant.
	// This does x = x - quadrant * PI / 2 using a two step Cody-Waite argument reduction.
	// This improves the accuracy of the result by avoiding loss of significant bits in the subtraction.
	// We start with x = x - quadrant * PI / 2, PI / 2 in hexadecimal notation is 0x3fc90fdb, we remove the lowest 16 bits to
	// get 0x3fc90000 (= 1.5703125) this means we can now multiply with a number of up to 2^16 without losing any bits.
	// This leaves us with: x = (x - quadrant * 1.5703125) - quadrant * (PI / 2 - 1.5703125).
	// PI / 2 - 1.5703125 in hexadecimal is 0x39fdaa22, stripping the lowest 12 bits we get 0x39fda000 (= 0.0004837512969970703125)
	// This leaves uw with: x = ((x - quadrant * 1.5703125) - quadrant * 0.0004837512969970703125) - quadrant * (PI / 2 - 1.5703125 - 0.0004837512969970703125)
	// See: https://stackoverflow.com/questions/42455143/sine-cosine-modular-extended-precision-arithmetic
	// After this we have x in the range [-PI / 4, PI / 4].
	Vec4 decimal_quadrant = quadrant.ToFloat();
	x = ((x - decimal_quadrant * decimal(1.5703125f)) - decimal_quadrant * decimal(0.0004837512969970703125f)) - decimal_quadrant * decimal(7.549789948768648e-8f);

	// Calculate x2 = x^2
	Vec4 x2 = x * x;

	// Taylor expansion:
	// Cos(x) = 1 - x^2/2! + x^4/4! - x^6/6! + x^8/8! + ... = (((x2/8!- 1/6!) * x2 + 1/4!) * x2 - 1/2!) * x2 + 1
	Vec4 taylor_cos = ((decimal(2.443315711809948e-5f) * x2 - Vec4::sReplicate(decimal(1.388731625493765e-3f))) * x2 + Vec4::sReplicate(decimal(4.166664568298827e-2f))) * x2 * x2 - C0P5 * x2 + Vec4::sReplicate(C1);
	// Sin(x) = x - x^3/3! + x^5/5! - x^7/7! + ... = ((-x2/7! + 1/5!) * x2 - 1/3!) * x2 * x + x
	Vec4 taylor_sin = ((decimal(- 1.9515295891e-4f) * x2 + Vec4::sReplicate(decimal(8.3321608736e-3f))) * x2 - Vec4::sReplicate(decimal(1.6666654611e-1f))) * x2 * x + x;

	// The lowest 2 bits of quadrant indicate the quadrant that we are in.
	// Let x be the original input value and x' our value that has been mapped to the range [-PI / 4, PI / 4].
	// since cos(x) = sin(x - PI / 2) and since we want to use the Taylor expansion as close as possible to 0,
	// we can alternate between using the Taylor expansion for sin and cos according to the following table:
	// 
	// quadrant	 sin(x)		 cos(x)
	// XXX00b	 sin(x')	 cos(x')
	// XXX01b	 cos(x')	-sin(x')
	// XXX10b	-sin(x')	-cos(x')
	// XXX11b	-cos(x')	 sin(x')
	//
	// So: sin_sign = bit2, cos_sign = bit1 ^ bit2, bit1 determines if we use sin or cos Taylor expansion
	UVec4 bit1 = quadrant.LogicalShiftLeft<31>();
	UVec4 bit2 = UVec4::sAnd(quadrant.LogicalShiftLeft<30>(), UVec4::sReplicate(0x80000000U));

	// Select which one of the results is sin and which one is cos
	Vec4 s = Vec4::sSelect(taylor_sin, taylor_cos, bit1);
	Vec4 c = Vec4::sSelect(taylor_cos, taylor_sin, bit1);

	// Update the signs
	sin_sign = UVec4::sXor(sin_sign, bit2);
	UVec4 cos_sign = UVec4::sXor(bit1, bit2);

	// Correct the signs
	outSin = Vec4::sXor(s, sin_sign.ReinterpretAsFloat());
	outCos = Vec4::sXor(c, cos_sign.ReinterpretAsFloat());
}

Vec4 Vec4::Tan() const
{
	// Implementation based on tanf.c from the cephes library, see Vec4::SinCos for further details
	// Original implementation by Stephen L. Moshier (See: http://www.moshier.net/)

	// Make argument positive
	UVec4 tan_sign = UVec4::sAnd(ReinterpretAsInt(), UVec4::sReplicate(0x80000000U));
	Vec4 x = Vec4::sXor(*this, tan_sign.ReinterpretAsFloat());

	// x / (PI / 2) rounded to nearest int gives us the quadrant closest to x
	UVec4 quadrant = (decimal(0.6366197723675814f) * x + Vec4::sReplicate(C0P5)).ToInt();

	// Remap x to range [-PI / 4, PI / 4], see Vec4::SinCos
	Vec4 decimal_quadrant = quadrant.ToFloat();
	x = ((x - decimal_quadrant * decimal(1.5703125f)) - decimal_quadrant * decimal(0.0004837512969970703125f)) - decimal_quadrant * decimal(7.549789948768648e-8f);

	// Calculate x2 = x^2	
	Vec4 x2 = x * x;

	// Roughly equivalent to the Taylor expansion:
	// Tan(x) = x + x^3/3 + 2*x^5/15 + 17*x^7/315 + 62*x^9/2835 + ...
	Vec4 tan =
		(((((decimal(9.38540185543e-3f) * x2 + Vec4::sReplicate(decimal(3.11992232697e-3f))) * x2 + Vec4::sReplicate(decimal(2.44301354525e-2f))) * x2
		+ Vec4::sReplicate(decimal(5.34112807005e-2f))) * x2 + Vec4::sReplicate(decimal(1.33387994085e-1f))) * x2 + Vec4::sReplicate(decimal(3.33331568548e-1f))) * x2 * x + x;

	// For the 2nd and 4th quadrant we need to invert the value
	UVec4 bit1 = quadrant.LogicalShiftLeft<31>();
	tan = Vec4::sSelect(tan, Vec4::sReplicate(-C1) / (tan JPH_IF_FLOATING_POINT_EXCEPTIONS_ENABLED(+ Vec4::sReplicate(FLT_MIN))), bit1); // Add small epsilon to prevent div by zero, works because tan is always positive

	// Put the sign back
	return Vec4::sXor(tan, tan_sign.ReinterpretAsFloat());
}

Vec4 Vec4::ASin() const
{
	// Implementation based on asinf.c from the cephes library
	// Original implementation by Stephen L. Moshier (See: http://www.moshier.net/)

	// Make argument positive
	UVec4 asin_sign = UVec4::sAnd(ReinterpretAsInt(), UVec4::sReplicate(0x80000000U));
	Vec4 a = Vec4::sXor(*this, asin_sign.ReinterpretAsFloat());

	// ASin is not defined outside the range [-1, 1] but it often happens that a value is slightly above 1 so we just clamp here
	a = Vec4::sMin(a, Vec4::sReplicate(C1));

	// When |x| <= 0.5 we use the asin approximation as is
	Vec4 z1 = a * a;
	Vec4 x1 = a;

	// When |x| > 0.5 we use the identity asin(x) = PI / 2 - 2 * asin(sqrt((1 - x) / 2))
	Vec4 z2 = C0P5 * (Vec4::sReplicate(C1) - a);
	Vec4 x2 = z2.Sqrt();

	// Select which of the two situations we have
	UVec4 greater = Vec4::sGreater(a, Vec4::sReplicate(C0P5));
	Vec4 z = Vec4::sSelect(z1, z2, greater);
	Vec4 x = Vec4::sSelect(x1, x2, greater);

	// Polynomial approximation of asin
	z = ((((decimal(4.2163199048e-2f) * z + Vec4::sReplicate(decimal(2.4181311049e-2f))) * z + Vec4::sReplicate(decimal(4.5470025998e-2f))) * z + Vec4::sReplicate(decimal(7.4953002686e-2f))) * z + Vec4::sReplicate(decimal(1.6666752422e-1f))) * z * x + x;

	// If |x| > 0.5 we need to apply the remainder of the identity above
	z = Vec4::sSelect(z, Vec4::sReplicate(C0P5 * JPH_PI) - (z + z), greater);

	// Put the sign back
	return Vec4::sXor(z, asin_sign.ReinterpretAsFloat());
}

Vec4 Vec4::ACos() const
{
	// Not the most accurate, but simple
	return Vec4::sReplicate(C0P5 * JPH_PI) - ASin();
}

Vec4 Vec4::ATan() const
{
	// Implementation based on atanf.c from the cephes library
	// Original implementation by Stephen L. Moshier (See: http://www.moshier.net/)

	// Make argument positive
	UVec4 atan_sign = UVec4::sAnd(ReinterpretAsInt(), UVec4::sReplicate(0x80000000U));
	Vec4 x = Vec4::sXor(*this, atan_sign.ReinterpretAsFloat());
	Vec4 y = Vec4::sZero();

	// If x > Tan(PI / 8)
	UVec4 greater1 = Vec4::sGreater(x, Vec4::sReplicate(decimal(0.4142135623730950f))); 
	Vec4 x1 = (x - Vec4::sReplicate(C1)) / (x + Vec4::sReplicate(C1));

	// If x > Tan(3 * PI / 8)
	UVec4 greater2 = Vec4::sGreater(x, Vec4::sReplicate(decimal(2.414213562373095f))); 
	Vec4 x2 = Vec4::sReplicate(-C1) / (x JPH_IF_FLOATING_POINT_EXCEPTIONS_ENABLED(+ Vec4::sReplicate(FLT_MIN))); // Add small epsilon to prevent div by zero, works because x is always positive

	// Apply first if
	x = Vec4::sSelect(x, x1, greater1);
	y = Vec4::sSelect(y, Vec4::sReplicate(decimal(0.25f) * JPH_PI), greater1);

	// Apply second if
	x = Vec4::sSelect(x, x2, greater2);
	y = Vec4::sSelect(y, Vec4::sReplicate(C0P5 * JPH_PI), greater2);

	// Polynomial approximation
	Vec4 z = x * x;
	y += (((decimal(8.05374449538e-2f) * z - Vec4::sReplicate(decimal(1.38776856032e-1f))) * z + Vec4::sReplicate(decimal(1.99777106478e-1f))) * z - Vec4::sReplicate(decimal(3.33329491539e-1f))) * z * x + x;

	// Put the sign back
	return Vec4::sXor(y, atan_sign.ReinterpretAsFloat());
}

Vec4 Vec4::sATan2(Vec4Arg inY, Vec4Arg inX)
{
	UVec4 sign_mask = UVec4::sReplicate(0x80000000U);

	// Determine absolute value and sign of y
	UVec4 y_sign = UVec4::sAnd(inY.ReinterpretAsInt(), sign_mask);
	Vec4 y_abs = Vec4::sXor(inY, y_sign.ReinterpretAsFloat());

	// Determine absolute value and sign of x
	UVec4 x_sign = UVec4::sAnd(inX.ReinterpretAsInt(), sign_mask);
	Vec4 x_abs = Vec4::sXor(inX, x_sign.ReinterpretAsFloat());

	// Always divide smallest / largest to avoid dividing by zero
	UVec4 x_is_numerator = Vec4::sLess(x_abs, y_abs);
	Vec4 numerator = Vec4::sSelect(y_abs, x_abs, x_is_numerator);
	Vec4 denominator = Vec4::sSelect(x_abs, y_abs, x_is_numerator);
	Vec4 atan = (numerator / denominator).ATan();

	// If we calculated x / y instead of y / x the result is PI / 2 - result (note that this is true because we know the result is positive because the input was positive)
	atan = Vec4::sSelect(atan, Vec4::sReplicate(C0P5 * JPH_PI) - atan, x_is_numerator);

	// Now we need to map to the correct quadrant
	// x_sign	y_sign	result
	// +1		+1		atan
	// -1		+1		-atan + PI
	// -1		-1		atan - PI
	// +1		-1		-atan
	// This can be written as: x_sign * y_sign * (atan - (x_sign < 0? PI : 0))
	atan -= Vec4::sAnd(x_sign.ArithmeticShiftRight<31>().ReinterpretAsFloat(), Vec4::sReplicate(JPH_PI));
	atan = Vec4::sXor(atan, UVec4::sXor(x_sign, y_sign).ReinterpretAsFloat());
	return atan;
}

JPH_NAMESPACE_END
