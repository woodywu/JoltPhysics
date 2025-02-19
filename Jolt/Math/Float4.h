// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

JPH_NAMESPACE_BEGIN

/// Class that holds 4 decimal values. Convert to Vec4 to perform calculations.
class [[nodiscard]] Float4
{
public:
	JPH_OVERRIDE_NEW_DELETE

				Float4() = default; ///< Intentionally not initialized for performance reasons
				Float4(const Float4 &inRHS) = default;
				Float4(decimal inX, decimal inY, decimal inZ, decimal inW) : x(inX), y(inY), z(inZ), w(inW) { }

	decimal		operator [] (int inCoordinate) const	
	{ 
		JPH_ASSERT(inCoordinate < 4); 
		return *(&x + inCoordinate); 
	}

	decimal		x;
	decimal		y;
	decimal		z;
	decimal		w;
};

static_assert(is_trivial<Float4>(), "Is supposed to be a trivial type!");

JPH_NAMESPACE_END
