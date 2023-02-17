// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Jolt/Math/Float2.h>

JPH_NAMESPACE_BEGIN

/// Ellipse centered around the origin
/// @see https://en.wikipedia.org/wiki/Ellipse
class Ellipse
{
public:
	JPH_OVERRIDE_NEW_DELETE

	/// Construct ellipse with radius A along the X-axis and B along the Y-axis
					Ellipse(decimal inA, decimal inB) : mA(inA), mB(inB) { JPH_ASSERT(inA > C0); JPH_ASSERT(inB > C0); }

	/// Check if inPoint is inside the ellipsse
	bool			IsInside(const Float2 &inPoint) const
	{
		return Square(inPoint.x / mA) + Square(inPoint.y / mB) <= C1;
	}

	/// Get the closest point on the ellipse to inPoint
	/// Assumes inPoint is outside the ellipse
	/// @see Rotation Joint Limits in Quaterion Space by Gino van den Bergen, section 10.1 in Game Engine Gems 3.
	Float2			GetClosestPoint(const Float2 &inPoint) const
	{
		decimal a_sq = Square(mA);
		decimal b_sq = Square(mB);

		// Equation of ellipse: f(x, y) = (x/a)^2 + (y/b)^2 - 1 = 0											[1]
		// Normal on surface: (df/dx, df/dy) = (2 x / a^2, 2 y / b^2)
		// Closest point (x', y') on ellipse to point (x, y): (x', y') + t (x / a^2, y / b^2) = (x, y)
		// <=> (x', y') = (a^2 x / (t + a^2), b^2 y / (t + b^2))
		// Requiring point to be on ellipse (substituting into [1]): g(t) = (a x / (t + a^2))^2 + (b y / (t + b^2))^2 - 1 = 0

		// Newton raphson iteration, starting at t = 0
		decimal t = C0;
		for (;;)
		{
			// Calculate g(t)
			decimal t_plus_a_sq = t + a_sq;
			decimal t_plus_b_sq = t + b_sq;
			decimal gt = Square(mA * inPoint.x / t_plus_a_sq) + Square(mB * inPoint.y / t_plus_b_sq) - C1;

			// Check if g(t) it is close enough to zero
			if (abs(gt) < decimal(1.0e-6f))
				return Float2(a_sq * inPoint.x / t_plus_a_sq, b_sq * inPoint.y / t_plus_b_sq);

			// Get derivative dg/dt = g'(t) = -2 (b^2 y^2 / (t + b^2)^3 + a^2 x^2 / (t + a^2)^3)
			decimal gt_accent = -C2 * 
				(a_sq * Square(inPoint.x) / Cubed(t_plus_a_sq) 
				+ b_sq * Square(inPoint.y) / Cubed(t_plus_b_sq));

			// Calculate t for next iteration: tn+1 = tn - g(t) / g'(t)
			decimal tn = t - gt / gt_accent;
			t = tn;			
		}
	}

	/// Get normal at point inPoint (non-normalized vector)
	Float2			GetNormal(const Float2 &inPoint) const
	{
		// Calculated by [d/dx f(x, y), d/dy f(x, y)], where f(x, y) is the ellipse equation from above
		return Float2(inPoint.x / Square(mA), inPoint.y / Square(mB));
	}

private:
	decimal			mA;				///< Radius along X-axis
	decimal			mB;				///< Radius along Y-axis
};

JPH_NAMESPACE_END
