// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Jolt/Math/Vec3.h>
#include <Jolt/Math/Vec4.h>
#include <Jolt/Math/Quat.h>

JPH_NAMESPACE_BEGIN

#define JPH_EL(r, c) mCol[c].mF32[r]

Mat44::Mat44(Vec4Arg inC1, Vec4Arg inC2, Vec4Arg inC3, Vec4Arg inC4) : 
	mCol { inC1, inC2, inC3, inC4 } 
{ 
}

Mat44::Mat44(Vec4Arg inC1, Vec4Arg inC2, Vec4Arg inC3, Vec3Arg inC4) : 
	mCol { inC1, inC2, inC3, Vec4(inC4, C1) }
{ 
}

Mat44::Mat44(Type inC1, Type inC2, Type inC3, Type inC4) : 
	mCol { inC1, inC2, inC3, inC4 } 
{
}

Mat44 Mat44::sZero()
{
	return Mat44(Vec4::sZero(), Vec4::sZero(), Vec4::sZero(), Vec4::sZero());
}

Mat44 Mat44::sIdentity()
{
	return Mat44(Vec4(C1, C0, C0, C0), Vec4(C0, C1, C0, C0), Vec4(C0, C0, C1, C0), Vec4(C0, C0, C0, C1));
}

Mat44 Mat44::sNaN()
{
	return Mat44(Vec4::sNaN(), Vec4::sNaN(), Vec4::sNaN(), Vec4::sNaN());
}

Mat44 Mat44::sLoadFloat4x4(const Float4 *inV)
{
	Mat44 result;
	for (int c = 0; c < 4; ++c)
		result.mCol[c] = Vec4::sLoadFloat4(inV + c);
	return result;
}

Mat44 Mat44::sLoadFloat4x4Aligned(const Float4 *inV)
{
	Mat44 result;
	for (int c = 0; c < 4; ++c)
		result.mCol[c] = Vec4::sLoadFloat4Aligned(inV + c);
	return result;
}

Mat44 Mat44::sRotationX(decimal inX)
{
	Vec4 sv, cv;
	Vec4::sReplicate(inX).SinCos(sv, cv);
	decimal s = sv.GetX(), c = cv.GetX();
	return Mat44(Vec4(C1, C0, C0, C0), Vec4(C0, c, s, C0), Vec4(C0, -s, c, C0), Vec4(C0, C0, C0, C1));
}

Mat44 Mat44::sRotationY(decimal inY)
{
	Vec4 sv, cv;
	Vec4::sReplicate(inY).SinCos(sv, cv);
	decimal s = sv.GetX(), c = cv.GetX();
	return Mat44(Vec4(c, C0, -s, C0), Vec4(C0, C1, C0, C0), Vec4(s, C0, c, C0), Vec4(C0, C0, C0, C1));
}

Mat44 Mat44::sRotationZ(decimal inZ)
{
	Vec4 sv, cv;
	Vec4::sReplicate(inZ).SinCos(sv, cv);
	decimal s = sv.GetX(), c = cv.GetX();
	return Mat44(Vec4(c, s, C0, C0), Vec4(-s, c, C0, C0), Vec4(C0, C0, C1, C0), Vec4(C0, C0, C0, C1));
}

Mat44 Mat44::sRotation(QuatArg inQuat)
{	
	JPH_ASSERT(inQuat.IsNormalized());

	decimal x = inQuat.GetX();
	decimal y = inQuat.GetY();
	decimal z = inQuat.GetZ();
	decimal w = inQuat.GetW();

	decimal tx = x + x; // Note: Using x + x instead of 2.0f * x to force this function to return the same value as the SSE4.1 version across platforms.
	decimal ty = y + y;
	decimal tz = z + z;

	decimal xx = tx * x;
	decimal yy = ty * y;
	decimal zz = tz * z;
	decimal xy = tx * y;
	decimal xz = tx * z;
	decimal xw = tx * w;
	decimal yz = ty * z;
	decimal yw = ty * w;
	decimal zw = tz * w;

	return Mat44(Vec4((C1 - yy) - zz, xy + zw, xz - yw, C0), // Note: Added extra brackets to force this function to return the same value as the SSE4.1 version across platforms.
				 Vec4(xy - zw, (C1 - zz) - xx, yz + xw, C0),
				 Vec4(xz + yw, yz - xw, (C1 - xx) - yy, C0),
				 Vec4(C0, C0, C0, C1));
}

Mat44 Mat44::sRotation(Vec3Arg inAxis, decimal inAngle)
{
	return sRotation(Quat::sRotation(inAxis, inAngle));
}

Mat44 Mat44::sTranslation(Vec3Arg inV)
{
	return Mat44(Vec4(C1, C0, C0, C0), Vec4(C0, C1, C0, C0), Vec4(C0, C0, C1, C0), Vec4(inV, C1));
}

Mat44 Mat44::sRotationTranslation(QuatArg inR, Vec3Arg inT)
{
	Mat44 m = sRotation(inR);
	m.SetTranslation(inT);
	return m;
}

Mat44 Mat44::sInverseRotationTranslation(QuatArg inR, Vec3Arg inT)
{
	Mat44 m = sRotation(inR.Conjugated());
	m.SetTranslation(-m.Multiply3x3(inT));
	return m;
}

Mat44 Mat44::sScale(decimal inScale)
{
	return Mat44(Vec4(inScale, C0, C0, C0), Vec4(C0, inScale, C0, C0), Vec4(C0, C0, inScale, C0), Vec4(C0, C0, C0, C1));
}

Mat44 Mat44::sScale(Vec3Arg inV)
{
	return Mat44(Vec4(inV.GetX(), C0, C0, C0), Vec4(C0, inV.GetY(), C0, C0), Vec4(C0, C0, inV.GetZ(), C0), Vec4(C0, C0, C0, C1));
}

Mat44 Mat44::sOuterProduct(Vec3Arg inV1, Vec3Arg inV2)
{
	Vec4 v1(inV1, C0);
	return Mat44(v1 * inV2.SplatX(), v1 * inV2.SplatY(), v1 * inV2.SplatZ(), Vec4(C0, C0, C0, C1));
}

Mat44 Mat44::sCrossProduct(Vec3Arg inV)
{
	decimal x = inV.GetX();
	decimal y = inV.GetY();
	decimal z = inV.GetZ();

	return Mat44(
		Vec4(C0, z, -y, C0),
		Vec4(-z, C0, x, C0),
		Vec4(y, -x, C0, C0),
		Vec4(C0, C0, C0, C1));
}

Mat44 Mat44::sLookAt(Vec3Arg inPos, Vec3Arg inTarget, Vec3Arg inUp)
{
	Vec3 direction = (inTarget - inPos).NormalizedOr(-Vec3::sAxisZ());
	Vec3 right = direction.Cross(inUp).NormalizedOr(Vec3::sAxisX());
	Vec3 up = right.Cross(direction);

	return Mat44(Vec4(right, C0), Vec4(up, C0), Vec4(-direction, C0), Vec4(inPos, C1)).InversedRotationTranslation();	
}

bool Mat44::operator == (Mat44Arg inM2) const
{
	return UVec4::sAnd(
		UVec4::sAnd(Vec4::sEquals(mCol[0], inM2.mCol[0]), Vec4::sEquals(mCol[1], inM2.mCol[1])),
		UVec4::sAnd(Vec4::sEquals(mCol[2], inM2.mCol[2]), Vec4::sEquals(mCol[3], inM2.mCol[3]))
	).TestAllTrue();
}

bool Mat44::IsClose(Mat44Arg inM2, decimal inMaxDistSq) const
{
	for (int i = 0; i < 4; ++i)
		if (!mCol[i].IsClose(inM2.mCol[i], inMaxDistSq))
			return false;
	return true;
}

Mat44 Mat44::operator * (Mat44Arg inM) const
{
	Mat44 result;
	for (int i = 0; i < 4; ++i)
		result.mCol[i] = mCol[0] * inM.mCol[i].mF32[0] + mCol[1] * inM.mCol[i].mF32[1] + mCol[2] * inM.mCol[i].mF32[2] + mCol[3] * inM.mCol[i].mF32[3];
	return result;
}

Vec3 Mat44::operator * (Vec3Arg inV) const
{
	return Vec3(
		mCol[0].mF32[0] * inV.mF32[0] + mCol[1].mF32[0] * inV.mF32[1] + mCol[2].mF32[0] * inV.mF32[2] + mCol[3].mF32[0], 
		mCol[0].mF32[1] * inV.mF32[0] + mCol[1].mF32[1] * inV.mF32[1] + mCol[2].mF32[1] * inV.mF32[2] + mCol[3].mF32[1], 
		mCol[0].mF32[2] * inV.mF32[0] + mCol[1].mF32[2] * inV.mF32[1] + mCol[2].mF32[2] * inV.mF32[2] + mCol[3].mF32[2]);
}

Vec4 Mat44::operator * (Vec4Arg inV) const
{
	return Vec4(
		mCol[0].mF32[0] * inV.mF32[0] + mCol[1].mF32[0] * inV.mF32[1] + mCol[2].mF32[0] * inV.mF32[2] + mCol[3].mF32[0] * inV.mF32[3], 
		mCol[0].mF32[1] * inV.mF32[0] + mCol[1].mF32[1] * inV.mF32[1] + mCol[2].mF32[1] * inV.mF32[2] + mCol[3].mF32[1] * inV.mF32[3], 
		mCol[0].mF32[2] * inV.mF32[0] + mCol[1].mF32[2] * inV.mF32[1] + mCol[2].mF32[2] * inV.mF32[2] + mCol[3].mF32[2] * inV.mF32[3], 
		mCol[0].mF32[3] * inV.mF32[0] + mCol[1].mF32[3] * inV.mF32[1] + mCol[2].mF32[3] * inV.mF32[2] + mCol[3].mF32[3] * inV.mF32[3]);
}

Vec3 Mat44::Multiply3x3(Vec3Arg inV) const
{
	return Vec3(
		mCol[0].mF32[0] * inV.mF32[0] + mCol[1].mF32[0] * inV.mF32[1] + mCol[2].mF32[0] * inV.mF32[2], 
		mCol[0].mF32[1] * inV.mF32[0] + mCol[1].mF32[1] * inV.mF32[1] + mCol[2].mF32[1] * inV.mF32[2], 
		mCol[0].mF32[2] * inV.mF32[0] + mCol[1].mF32[2] * inV.mF32[1] + mCol[2].mF32[2] * inV.mF32[2]);
}

Vec3 Mat44::Multiply3x3Transposed(Vec3Arg inV) const
{
	return Transposed3x3().Multiply3x3(inV);
}

Mat44 Mat44::Multiply3x3(Mat44Arg inM) const
{
	JPH_ASSERT(mCol[0][3] == C0);
	JPH_ASSERT(mCol[1][3] == C0);
	JPH_ASSERT(mCol[2][3] == C0);

	Mat44 result;
	for (int i = 0; i < 3; ++i)
		result.mCol[i] = mCol[0] * inM.mCol[i].mF32[0] + mCol[1] * inM.mCol[i].mF32[1] + mCol[2] * inM.mCol[i].mF32[2];
	result.mCol[3] = Vec4(C0, C0, C0, C1);
	return result;
}

Mat44 Mat44::Multiply3x3LeftTransposed(Mat44Arg inM) const
{
	// Transpose left hand side
	Mat44 trans = Transposed3x3();

	// Do 3x3 matrix multiply
	Mat44 result;
	result.mCol[0] = trans.mCol[0] * inM.mCol[0].SplatX() + trans.mCol[1] * inM.mCol[0].SplatY() + trans.mCol[2] * inM.mCol[0].SplatZ();
	result.mCol[1] = trans.mCol[0] * inM.mCol[1].SplatX() + trans.mCol[1] * inM.mCol[1].SplatY() + trans.mCol[2] * inM.mCol[1].SplatZ();
	result.mCol[2] = trans.mCol[0] * inM.mCol[2].SplatX() + trans.mCol[1] * inM.mCol[2].SplatY() + trans.mCol[2] * inM.mCol[2].SplatZ();
	result.mCol[3] = Vec4(C0, C0, C0, C1);
	return result;
}

Mat44 Mat44::Multiply3x3RightTransposed(Mat44Arg inM) const
{
	JPH_ASSERT(mCol[0][3] == C0);
	JPH_ASSERT(mCol[1][3] == C0);
	JPH_ASSERT(mCol[2][3] == C0);

	Mat44 result;
	result.mCol[0] = mCol[0] * inM.mCol[0].SplatX() + mCol[1] * inM.mCol[1].SplatX() + mCol[2] * inM.mCol[2].SplatX();
	result.mCol[1] = mCol[0] * inM.mCol[0].SplatY() + mCol[1] * inM.mCol[1].SplatY() + mCol[2] * inM.mCol[2].SplatY();
	result.mCol[2] = mCol[0] * inM.mCol[0].SplatZ() + mCol[1] * inM.mCol[1].SplatZ() + mCol[2] * inM.mCol[2].SplatZ();
	result.mCol[3] = Vec4(C0, C0, C0, C1);
	return result;
}

Mat44 Mat44::operator * (decimal inV) const
{
	Vec4 multiplier = Vec4::sReplicate(inV);

	Mat44 result;
	for (int c = 0; c < 4; ++c)
		result.mCol[c] = mCol[c] * multiplier;
	return result;
}

Mat44 &Mat44::operator *= (decimal inV)
{
	for (int c = 0; c < 4; ++c)
		mCol[c] *= inV;

	return *this;
}

Mat44 Mat44::operator + (Mat44Arg inM) const
{
	Mat44 result;
	for (int i = 0; i < 4; ++i)
		result.mCol[i] = mCol[i] + inM.mCol[i];
	return result;
}

Mat44 Mat44::operator - () const
{
	Mat44 result;
	for (int i = 0; i < 4; ++i)
		result.mCol[i] = -mCol[i];
	return result;
}

Mat44 Mat44::operator - (Mat44Arg inM) const
{
	Mat44 result;
	for (int i = 0; i < 4; ++i)
		result.mCol[i] = mCol[i] - inM.mCol[i];
	return result;
}

Mat44 &Mat44::operator += (Mat44Arg inM)
{
	for (int c = 0; c < 4; ++c)
		mCol[c] += inM.mCol[c];

	return *this;
}

void Mat44::StoreFloat4x4(Float4 *outV) const
{
	for (int c = 0; c < 4; ++c)
		mCol[c].StoreFloat4(outV + c);
}

Mat44 Mat44::Transposed() const
{
	Mat44 result;
	for (int c = 0; c < 4; ++c)
		for (int r = 0; r < 4; ++r)
			result.mCol[r].mF32[c] = mCol[c].mF32[r];
	return result;
}

Mat44 Mat44::Transposed3x3() const
{
	Mat44 result;
	for (int c = 0; c < 3; ++c)
	{
		for (int r = 0; r < 3; ++r)
			result.mCol[c].mF32[r] = mCol[r].mF32[c];
		result.mCol[c].mF32[3] = C0;
	}
	result.mCol[3] = Vec4(C0, C0, C0, C1);
	return result;
}

Mat44 Mat44::Inversed() const
{
	decimal m00 = JPH_EL(0, 0), m10 = JPH_EL(1, 0), m20 = JPH_EL(2, 0), m30 = JPH_EL(3, 0);
	decimal m01 = JPH_EL(0, 1), m11 = JPH_EL(1, 1), m21 = JPH_EL(2, 1), m31 = JPH_EL(3, 1);
	decimal m02 = JPH_EL(0, 2), m12 = JPH_EL(1, 2), m22 = JPH_EL(2, 2), m32 = JPH_EL(3, 2);
	decimal m03 = JPH_EL(0, 3), m13 = JPH_EL(1, 3), m23 = JPH_EL(2, 3), m33 = JPH_EL(3, 3);
	
	decimal m10211120 = m10 * m21 - m11 * m20;
	decimal m10221220 = m10 * m22 - m12 * m20;
	decimal m10231320 = m10 * m23 - m13 * m20;
	decimal m10311130 = m10 * m31 - m11 * m30;
	decimal m10321230 = m10 * m32 - m12 * m30;
	decimal m10331330 = m10 * m33 - m13 * m30;
	decimal m11221221 = m11 * m22 - m12 * m21;
	decimal m11231321 = m11 * m23 - m13 * m21;
	decimal m11321231 = m11 * m32 - m12 * m31;
	decimal m11331331 = m11 * m33 - m13 * m31;
	decimal m12231322 = m12 * m23 - m13 * m22;
	decimal m12331332 = m12 * m33 - m13 * m32;
	decimal m20312130 = m20 * m31 - m21 * m30;
	decimal m20322230 = m20 * m32 - m22 * m30;
	decimal m20332330 = m20 * m33 - m23 * m30;
	decimal m21322231 = m21 * m32 - m22 * m31;
	decimal m21332331 = m21 * m33 - m23 * m31;
	decimal m22332332 = m22 * m33 - m23 * m32;

	Vec4 col0(m11 * m22332332 - m12 * m21332331 + m13 * m21322231,		-m10 * m22332332 + m12 * m20332330 - m13 * m20322230,		m10 * m21332331 - m11 * m20332330 + m13 * m20312130,		-m10 * m21322231 + m11 * m20322230 - m12 * m20312130);
	Vec4 col1(-m01 * m22332332 + m02 * m21332331 - m03 * m21322231,		m00 * m22332332 - m02 * m20332330 + m03 * m20322230,		-m00 * m21332331 + m01 * m20332330 - m03 * m20312130,		m00 * m21322231 - m01 * m20322230 + m02 * m20312130);
	Vec4 col2(m01 * m12331332 - m02 * m11331331 + m03 * m11321231,		-m00 * m12331332 + m02 * m10331330 - m03 * m10321230,		m00 * m11331331 - m01 * m10331330 + m03 * m10311130,		-m00 * m11321231 + m01 * m10321230 - m02 * m10311130);
	Vec4 col3(-m01 * m12231322 + m02 * m11231321 - m03 * m11221221,		m00 * m12231322 - m02 * m10231320 + m03 * m10221220,		-m00 * m11231321 + m01 * m10231320 - m03 * m10211120,		m00 * m11221221 - m01 * m10221220 + m02 * m10211120);

	decimal det = m00 * col0.mF32[0] + m01 * col0.mF32[1] + m02 * col0.mF32[2] + m03 * col0.mF32[3];

	return Mat44(col0 / det, col1 / det, col2 / det, col3 / det);
}

Mat44 Mat44::InversedRotationTranslation() const
{
	Mat44 m = Transposed3x3();
	m.SetTranslation(-m.Multiply3x3(GetTranslation()));
	return m;
}

decimal Mat44::GetDeterminant3x3() const
{
	return GetAxisX().Dot(GetAxisY().Cross(GetAxisZ()));
}

Mat44 Mat44::Adjointed3x3() const
{
	// Adapted from Inversed() to remove 4th column and the division by the determinant
	// Note: This can be optimized.

	JPH_ASSERT(mCol[0][3] == C0);
	JPH_ASSERT(mCol[1][3] == C0);
	JPH_ASSERT(mCol[2][3] == C0);

	return Mat44(
		Vec4(JPH_EL(1, 1) * JPH_EL(2, 2) - JPH_EL(1, 2) * JPH_EL(2, 1),
			JPH_EL(1, 2) * JPH_EL(2, 0) - JPH_EL(1, 0) * JPH_EL(2, 2),
			JPH_EL(1, 0) * JPH_EL(2, 1) - JPH_EL(1, 1) * JPH_EL(2, 0),
			C0),
		Vec4(JPH_EL(0, 2) * JPH_EL(2, 1) - JPH_EL(0, 1) * JPH_EL(2, 2),
			JPH_EL(0, 0) * JPH_EL(2, 2) - JPH_EL(0, 2) * JPH_EL(2, 0),
			JPH_EL(0, 1) * JPH_EL(2, 0) - JPH_EL(0, 0) * JPH_EL(2, 1),
			C0),
		Vec4(JPH_EL(0, 1) * JPH_EL(1, 2) - JPH_EL(0, 2) * JPH_EL(1, 1),
			JPH_EL(0, 2) * JPH_EL(1, 0) - JPH_EL(0, 0) * JPH_EL(1, 2),
			JPH_EL(0, 0) * JPH_EL(1, 1) - JPH_EL(0, 1) * JPH_EL(1, 0),
			C0),
		Vec4(C0, C0, C0, C1));
}

Mat44 Mat44::Inversed3x3() const
{
	// Adapted from Inversed() to remove 4th column
	// Note: This can be optimized.

	JPH_ASSERT(mCol[0][3] == C0);
	JPH_ASSERT(mCol[1][3] == C0);
	JPH_ASSERT(mCol[2][3] == C0);

	decimal det = GetDeterminant3x3();

	return Mat44(
		Vec4((JPH_EL(1, 1) * JPH_EL(2, 2) - JPH_EL(1, 2) * JPH_EL(2, 1)) / det,
			(JPH_EL(1, 2) * JPH_EL(2, 0) - JPH_EL(1, 0) * JPH_EL(2, 2)) / det,
			(JPH_EL(1, 0) * JPH_EL(2, 1) - JPH_EL(1, 1) * JPH_EL(2, 0)) / det,
			C0),
		Vec4((JPH_EL(0, 2) * JPH_EL(2, 1) - JPH_EL(0, 1) * JPH_EL(2, 2)) / det,
			(JPH_EL(0, 0) * JPH_EL(2, 2) - JPH_EL(0, 2) * JPH_EL(2, 0)) / det,
			(JPH_EL(0, 1) * JPH_EL(2, 0) - JPH_EL(0, 0) * JPH_EL(2, 1)) / det,
			C0),
		Vec4((JPH_EL(0, 1) * JPH_EL(1, 2) - JPH_EL(0, 2) * JPH_EL(1, 1)) / det,
			(JPH_EL(0, 2) * JPH_EL(1, 0) - JPH_EL(0, 0) * JPH_EL(1, 2)) / det,
			(JPH_EL(0, 0) * JPH_EL(1, 1) - JPH_EL(0, 1) * JPH_EL(1, 0)) / det,
			C0),
		Vec4(C0, C0, C0, C1));
}

Quat Mat44::GetQuaternion() const
{
	JPH_ASSERT(mCol[3] == Vec4(C0, C0, C0, C1));

	decimal tr = mCol[0].mF32[0] + mCol[1].mF32[1] + mCol[2].mF32[2];

	if (tr >= C0)
	{
		decimal s = sqrt(tr + C1);
		decimal is = C0P5 / s;
		return Quat(
			(mCol[1].mF32[2] - mCol[2].mF32[1]) * is,
			(mCol[2].mF32[0] - mCol[0].mF32[2]) * is,
			(mCol[0].mF32[1] - mCol[1].mF32[0]) * is,
			C0P5 * s);
	}
	else
	{
		int i = 0;
		if (mCol[1].mF32[1] > mCol[0].mF32[0]) i = 1;
		if (mCol[2].mF32[2] > mCol[i].mF32[i]) i = 2;

		if (i == 0)
		{
			decimal s = sqrt(mCol[0].mF32[0] - (mCol[1].mF32[1] + mCol[2].mF32[2]) + 1);
			decimal is = C0P5 / s;
			return Quat(
				C0P5 * s,
				(mCol[1].mF32[0] + mCol[0].mF32[1]) * is,
				(mCol[0].mF32[2] + mCol[2].mF32[0]) * is,
				(mCol[1].mF32[2] - mCol[2].mF32[1]) * is);
		}
		else if (i == 1)
		{
			decimal s = sqrt(mCol[1].mF32[1] - (mCol[2].mF32[2] + mCol[0].mF32[0]) + 1);
			decimal is = C0P5 / s;
			return Quat(
				(mCol[1].mF32[0] + mCol[0].mF32[1]) * is,
				C0P5 * s,
				(mCol[2].mF32[1] + mCol[1].mF32[2]) * is,
				(mCol[2].mF32[0] - mCol[0].mF32[2]) * is);
		}
		else
		{
			JPH_ASSERT(i == 2);

			decimal s = sqrt(mCol[2].mF32[2] - (mCol[0].mF32[0] + mCol[1].mF32[1]) + 1);
			decimal is = C0P5 / s;
			return Quat(
				(mCol[0].mF32[2] + mCol[2].mF32[0]) * is,
				(mCol[2].mF32[1] + mCol[1].mF32[2]) * is,
				C0P5 * s,
				(mCol[0].mF32[1] - mCol[1].mF32[0]) * is);
		}
	}
}

Mat44 Mat44::sQuatLeftMultiply(QuatArg inQ)
{
	return Mat44(
		Vec4(C1, C1, -C1, -C1) * inQ.mValue.Swizzle<SWIZZLE_W, SWIZZLE_Z, SWIZZLE_Y, SWIZZLE_X>(),
		Vec4(-C1, C1, C1, -C1) * inQ.mValue.Swizzle<SWIZZLE_Z, SWIZZLE_W, SWIZZLE_X, SWIZZLE_Y>(),
		Vec4(C1, -C1, C1, -C1) * inQ.mValue.Swizzle<SWIZZLE_Y, SWIZZLE_X, SWIZZLE_W, SWIZZLE_Z>(),
		inQ.mValue);
}

Mat44 Mat44::sQuatRightMultiply(QuatArg inQ)
{
	return Mat44(
		Vec4(C1, -C1, C1, -C1) * inQ.mValue.Swizzle<SWIZZLE_W, SWIZZLE_Z, SWIZZLE_Y, SWIZZLE_X>(),
		Vec4(C1, C1, -C1, -C1) * inQ.mValue.Swizzle<SWIZZLE_Z, SWIZZLE_W, SWIZZLE_X, SWIZZLE_Y>(),
		Vec4(-C1, C1, C1, -C1) * inQ.mValue.Swizzle<SWIZZLE_Y, SWIZZLE_X, SWIZZLE_W, SWIZZLE_Z>(),
		inQ.mValue);
}

Mat44 Mat44::GetRotation() const
{ 
	JPH_ASSERT(mCol[0][3] == C0);
	JPH_ASSERT(mCol[1][3] == C0);
	JPH_ASSERT(mCol[2][3] == C0);

	return Mat44(mCol[0], mCol[1], mCol[2], Vec4(C0, C0, C0, C1)); 
}

Mat44 Mat44::GetRotationSafe() const
{ 
	return Mat44(Vec4(mCol[0].mF32[0], mCol[0].mF32[1], mCol[0].mF32[2], C0),
				 Vec4(mCol[1].mF32[0], mCol[1].mF32[1], mCol[1].mF32[2], C0),
				 Vec4(mCol[2].mF32[0], mCol[2].mF32[1], mCol[2].mF32[2], C0),
				 Vec4(C0, C0, C0, C1));
}

void Mat44::SetRotation(Mat44Arg inRotation)
{
	mCol[0] = inRotation.mCol[0];
	mCol[1] = inRotation.mCol[1];
	mCol[2] = inRotation.mCol[2];
}

Mat44 Mat44::PreTranslated(Vec3Arg inTranslation) const
{
	return Mat44(mCol[0], mCol[1], mCol[2], Vec4(GetTranslation() + Multiply3x3(inTranslation), C1));
}

Mat44 Mat44::PostTranslated(Vec3Arg inTranslation) const
{
	return Mat44(mCol[0], mCol[1], mCol[2], Vec4(GetTranslation() + inTranslation, C1));
}

Mat44 Mat44::PreScaled(Vec3Arg inScale) const
{
	return Mat44(inScale.GetX() * mCol[0], inScale.GetY() * mCol[1], inScale.GetZ() * mCol[2], mCol[3]);
}

Mat44 Mat44::PostScaled(Vec3Arg inScale) const
{
	Vec4 scale(inScale, C1);
	return Mat44(scale * mCol[0], scale * mCol[1], scale * mCol[2], scale * mCol[3]);
}

Mat44 Mat44::Decompose(Vec3 &outScale) const
{
	// Start the modified Gram-Schmidt algorithm
	// X axis will just be normalized
	Vec3 x = GetAxisX();

	// Make Y axis perpendicular to X
	Vec3 y = GetAxisY();
	decimal x_dot_x = x.LengthSq();
	y -= (x.Dot(y) / x_dot_x) * x;

	// Make Z axis perpendicular to X
	Vec3 z = GetAxisZ();
	z -= (x.Dot(z) / x_dot_x) * x;

	// Make Z axis perpendicular to Y
	decimal y_dot_y = y.LengthSq();
	z -= (y.Dot(z) / y_dot_y) * y;
	
	// Determine the scale
	decimal z_dot_z = z.LengthSq();
	outScale = Vec3(x_dot_x, y_dot_y, z_dot_z).Sqrt();

	// If the resulting x, y and z vectors don't form a right handed matrix, flip the z axis.
	if (x.Cross(y).Dot(z) < C0)
		outScale.SetZ(-outScale.GetZ());

	// Determine the rotation and translation
	return Mat44(Vec4(x / outScale.GetX(), C0), Vec4(y / outScale.GetY(), C0), Vec4(z / outScale.GetZ(), C0), GetColumn4(3));
}

#undef JPH_EL

JPH_NAMESPACE_END
