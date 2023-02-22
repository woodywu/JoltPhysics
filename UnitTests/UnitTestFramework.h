// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Jolt/Jolt.h>
#include <Jolt/Math/Float2.h>

// Disable common warnings
JPH_SUPPRESS_WARNINGS
JPH_CLANG_SUPPRESS_WARNING("-Wheader-hygiene")
#ifdef JPH_DOUBLE_PRECISION
JPH_CLANG_SUPPRESS_WARNING("-Wdouble-promotion")
#endif // JPH_DOUBLE_PRECISION

JPH_SUPPRESS_WARNINGS_STD_BEGIN
#include "doctest.h"
JPH_SUPPRESS_WARNINGS_STD_END

using namespace JPH;
using namespace JPH::literals;
using namespace std;

inline void CHECK_APPROX_EQUAL(float inLHS, float inRHS, float inTolerance = 1.0e-6f)
{
	CHECK(abs(inRHS - inLHS) <= inTolerance);
}

inline void CHECK_APPROX_EQUAL(double inLHS, double inRHS, double inTolerance = 1.0e-6)
{
	CHECK(abs(inRHS - inLHS) <= inTolerance);
}

// Define the exact random number generator we want to use across platforms for consistency (default_random_engine's implementation is platform specific)
using UnitTestRandom = mt19937;
