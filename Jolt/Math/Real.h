// SPDX-FileCopyrightText: 2022 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

JPH_NAMESPACE_BEGIN

// Define real to decimal
using Real = decimal;
using Real3 = Float3;
using RVec3  = Vec3;
using RVec3Arg = Vec3Arg;
using RMat44 = Mat44;
using RMat44Arg = Mat44Arg;

#define JPH_RVECTOR_ALIGNMENT JPH_VECTOR_ALIGNMENT

// Put the 'real' operator in a namespace so that users can opt in to use it:
// using namespace JPH::literals;
namespace literals {
	constexpr Real operator "" _r (long double inValue) { return Real(inValue); }
};

JPH_NAMESPACE_END
