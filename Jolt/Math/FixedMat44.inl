// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Jolt/Math/FixedVec3.h>

JPH_NAMESPACE_BEGIN

FixedMat44::FixedMat44(Vec4Arg inC1, Vec4Arg inC2, Vec4Arg inC3, FixedVec3Arg inC4) :
	mCol{ inC1, inC2, inC3 },
	mCol3(inC4) {}

FixedMat44::FixedMat44(Type inC1, Type inC2, Type inC3, FixedTypeArg inC4) :
	mCol{ inC1, inC2, inC3 },
	mCol3(inC4) {}

FixedMat44::FixedMat44(Mat44Arg inM) :
	mCol{ inM.GetColumn4(0), inM.GetColumn4(1), inM.GetColumn4(2) },
	mCol3(inM.GetTranslation()) {}

FixedMat44::FixedMat44(Mat44Arg inRot, FixedVec3Arg inT) :
	mCol{ inRot.GetColumn4(0), inRot.GetColumn4(1), inRot.GetColumn4(2) },
	mCol3(inT) {}

FixedMat44 FixedMat44::sZero() {
	return FixedMat44(Vec4::sZero(), Vec4::sZero(), Vec4::sZero(), FixedVec3::sZero());
}

FixedMat44 FixedMat44::sIdentity() {
	return FixedMat44(Vec4(1, 0, 0, 0), Vec4(0, 1, 0, 0), Vec4(0, 0, 1, 0), FixedVec3::sZero());
}

FixedMat44 FixedMat44::sInverseRotationTranslation(QuatArg inR, FixedVec3Arg inT) {
	Mat44 m = Mat44::sRotation(inR.Conjugated());
	FixedMat44 dm(m, FixedVec3::sZero());
	dm.SetTranslation(-dm.Multiply3x3(inT));
	return dm;
}

bool FixedMat44::operator == (FixedMat44Arg inM2) const {
	return mCol[0] == inM2.mCol[0]
		&& mCol[1] == inM2.mCol[1]
		&& mCol[2] == inM2.mCol[2]
		&& mCol3 == inM2.mCol3;
}

bool FixedMat44::IsClose(FixedMat44Arg inM2, float inMaxDistSq) const {
	for (int i = 0; i < 3; ++i)
		if (!mCol[i].IsClose(inM2.mCol[i], inMaxDistSq))
			return false;
	return mCol3.IsClose(inM2.mCol3, decimal(inMaxDistSq));
}

FixedVec3 FixedMat44::operator * (Vec3Arg inV) const {

	return FixedVec3(
		mCol3.mF32[0] + decimal(mCol[0].mF32[0] * inV.mF32[0] + mCol[1].mF32[0] * inV.mF32[1] + mCol[2].mF32[0] * inV.mF32[2]),
		mCol3.mF32[1] + decimal(mCol[0].mF32[1] * inV.mF32[0] + mCol[1].mF32[1] * inV.mF32[1] + mCol[2].mF32[1] * inV.mF32[2]),
		mCol3.mF32[2] + decimal(mCol[0].mF32[2] * inV.mF32[0] + mCol[1].mF32[2] * inV.mF32[1] + mCol[2].mF32[2] * inV.mF32[2]));
}

FixedVec3 FixedMat44::operator * (FixedVec3Arg inV) const {

	return FixedVec3(
		mCol3.mF32[0] + decimal(mCol[0].mF32[0]) * inV.mF32[0] + decimal(mCol[1].mF32[0]) * inV.mF32[1] + decimal(mCol[2].mF32[0]) * inV.mF32[2],
		mCol3.mF32[1] + decimal(mCol[0].mF32[1]) * inV.mF32[0] + decimal(mCol[1].mF32[1]) * inV.mF32[1] + decimal(mCol[2].mF32[1]) * inV.mF32[2],
		mCol3.mF32[2] + decimal(mCol[0].mF32[2]) * inV.mF32[0] + decimal(mCol[1].mF32[2]) * inV.mF32[1] + decimal(mCol[2].mF32[2]) * inV.mF32[2]);
}

FixedVec3 FixedMat44::Multiply3x3(FixedVec3Arg inV) const {

	return FixedVec3(
		decimal(mCol[0].mF32[0]) * inV.mF32[0] + decimal(mCol[1].mF32[0]) * inV.mF32[1] + decimal(mCol[2].mF32[0]) * inV.mF32[2],
		decimal(mCol[0].mF32[1]) * inV.mF32[0] + decimal(mCol[1].mF32[1]) * inV.mF32[1] + decimal(mCol[2].mF32[1]) * inV.mF32[2],
		decimal(mCol[0].mF32[2]) * inV.mF32[0] + decimal(mCol[1].mF32[2]) * inV.mF32[1] + decimal(mCol[2].mF32[2]) * inV.mF32[2]);
}

FixedMat44 FixedMat44::operator * (Mat44Arg inM) const {
	FixedMat44 result;

	// Rotation part
	for (int i = 0; i < 3; ++i) {
		Vec4 coli = inM.GetColumn4(i);
		result.mCol[i] = mCol[0] * coli.mF32[0] + mCol[1] * coli.mF32[1] + mCol[2] * coli.mF32[2];
	}

	// Translation part
	result.mCol3 = *this * inM.GetTranslation();

	return result;
}

FixedMat44 FixedMat44::operator * (FixedMat44Arg inM) const {
	FixedMat44 result;

	// Rotation part
	for (int i = 0; i < 3; ++i) {
		Vec4 coli = inM.mCol[i];
		result.mCol[i] = mCol[0] * coli.mF32[0] + mCol[1] * coli.mF32[1] + mCol[2] * coli.mF32[2];
	}

	// Translation part
	result.mCol3 = *this * inM.GetTranslation();

	return result;
}

void FixedMat44::SetRotation(Mat44Arg inRotation) {
	mCol[0] = inRotation.GetColumn4(0);
	mCol[1] = inRotation.GetColumn4(1);
	mCol[2] = inRotation.GetColumn4(2);
}

FixedMat44 FixedMat44::PreScaled(Vec3Arg inScale) const {
	return FixedMat44(inScale.GetX() * mCol[0], inScale.GetY() * mCol[1], inScale.GetZ() * mCol[2], mCol3);
}

FixedMat44 FixedMat44::PostScaled(Vec3Arg inScale) const {
	Vec4 scale(inScale, 1);
	return FixedMat44(scale * mCol[0], scale * mCol[1], scale * mCol[2], FixedVec3(scale) * mCol3);
}

FixedMat44 FixedMat44::PreTranslated(Vec3Arg inTranslation) const {
	return FixedMat44(mCol[0], mCol[1], mCol[2], GetTranslation() + Multiply3x3(inTranslation));
}

FixedMat44 FixedMat44::PreTranslated(FixedVec3Arg inTranslation) const {
	return FixedMat44(mCol[0], mCol[1], mCol[2], GetTranslation() + Multiply3x3(inTranslation));
}

FixedMat44 FixedMat44::PostTranslated(FixedVec3Arg inTranslation) const {
	return FixedMat44(mCol[0], mCol[1], mCol[2], GetTranslation() + inTranslation);
}

FixedMat44 FixedMat44::PostTranslated(FixedVec3Arg inTranslation) const {
	return FixedMat44(mCol[0], mCol[1], mCol[2], GetTranslation() + inTranslation);
}

FixedMat44 FixedMat44::Inversed() const {
	FixedMat44 m(GetRotation().Inversed3x3());
	m.mCol3 = -m.Multiply3x3(mCol3);
	return m;
}

FixedMat44 FixedMat44::InversedRotationTranslation() const {
	FixedMat44 m(GetRotation().Transposed3x3());
	m.mCol3 = -m.Multiply3x3(mCol3);
	return m;
}

JPH_NAMESPACE_END
