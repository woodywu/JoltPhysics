// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Jolt/Core/HashCombine.h>

JPH_NAMESPACE_BEGIN

/// Class that holds 3 decimals. Used as a storage class. Convert to Vec3 for calculations.
class [[nodiscard]] Fixed3
{
public:
	JPH_OVERRIDE_NEW_DELETE

				Fixed3() = default; ///< Intentionally not initialized for performance reasons
				Fixed3(const Fixed3 &inRHS) = default;
				Fixed3(decimal inX, decimal inY, decimal inZ) : x(inX), y(inY), z(inZ) { }

				decimal		operator [] (int inCoordinate) const
	{ 
		JPH_ASSERT(inCoordinate < 3); 
		return *(&x + inCoordinate); 
	}

	bool		operator == (const Fixed3 &inRHS) const
	{
		return x == inRHS.x && y == inRHS.y && z == inRHS.z;
	}

	bool		operator != (const Fixed3 &inRHS) const
	{
		return x != inRHS.x || y != inRHS.y || z != inRHS.z;
	}

	decimal		x;
	decimal		y;
	decimal		z;
};

using VertexList = Array<Fixed3>;

static_assert(is_trivial<Fixed3>(), "Is supposed to be a trivial type!");

JPH_NAMESPACE_END

// Create a std::hash for Fixed3
JPH_MAKE_HASHABLE(JPH::Fixed3, t.x, t.y, t.z)
