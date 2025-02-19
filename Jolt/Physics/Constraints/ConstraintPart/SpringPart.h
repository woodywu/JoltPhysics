// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

JPH_NAMESPACE_BEGIN
#ifndef JPH_PLATFORM_DOXYGEN // Somehow Doxygen gets confused and thinks the parameters to CalculateSpringProperties belong to this macro
JPH_MSVC_SUPPRESS_WARNING(4723) // potential divide by 0 - caused by line: outEffectiveMass = decimal(1.0f) / inInvEffectiveMass, note that JPH_NAMESPACE_BEGIN already pushes the warning state
#endif // !JPH_PLATFORM_DOXYGEN

/// Class used in other constraint parts to calculate the required bias factor in the lagrange multiplier for creating springs
class SpringPart
{
public:
	/// Calculate spring properties
	///
	/// @param inDeltaTime Time step
	/// @param inInvEffectiveMass Inverse effective mass K
	/// @param inBias Bias term (b) for the constraint impulse: lambda = J v + b
	///	@param inC Value of the constraint equation (C). Set to zero if you don't want to drive the constraint to zero with a spring.
	///	@param inFrequency Oscillation frequency (Hz). Set to zero if you don't want to drive the constraint to zero with a spring.
	///	@param inDamping Damping factor (0 = no damping, 1 = critical damping). Set to zero if you don't want to drive the constraint to zero with a spring.
	/// @param outEffectiveMass On return, this contains the new effective mass K^-1
	inline void					CalculateSpringProperties(decimal inDeltaTime, decimal inInvEffectiveMass, decimal inBias, decimal inC, decimal inFrequency, decimal inDamping, decimal &outEffectiveMass)
	{
		outEffectiveMass = decimal(1.0f) / inInvEffectiveMass;

		// Soft constraints as per: Soft Contraints: Reinventing The Spring - Erin Catto - GDC 2011
		if (inFrequency > decimal(0.0f))
		{
			// Calculate angular frequency
			decimal omega = decimal(2.0f) * JPH_PI * inFrequency;

			// Calculate spring constant k and drag constant c (page 45)
			decimal k = outEffectiveMass * Square(omega);
			decimal c = decimal(2.0f) * outEffectiveMass * inDamping * omega;

			// Note that the calculation of beta and gamma below are based on the solution of an implicit Euler integration scheme
			// This scheme is unconditionally stable but has built in damping, so even when you set the damping ratio to 0 there will still
			// be damping. See page 16 and 32.

			// Calculate softness (gamma in the slides)
			// See page 34 and note that the gamma needs to be divided by delta time since we're working with impulses rather than forces:
			// softness = 1 / (dt * (c + dt * k))
			mSoftness = decimal(1.0f) / (inDeltaTime * (c + inDeltaTime * k));

			// Calculate bias factor (baumgarte stabilization):
			// beta = dt * k / (c + dt * k) = dt * k^2 * softness
			// b = beta / dt * C = dt * k * softness * C;
			mBias = inBias + inDeltaTime * k * mSoftness * inC;
			
			// Update the effective mass, see post by Erin Catto: http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=4&t=1354
			// 
			// Newton's Law: 
			// M * (v2 - v1) = J^T * lambda 
			// 
			// Velocity constraint with softness and Baumgarte: 
			// J * v2 + softness * lambda + b = 0 
			// 
			// where b = beta * C / dt 
			// 
			// We know everything except v2 and lambda. 
			// 
			// First solve Newton's law for v2 in terms of lambda: 
			// 
			// v2 = v1 + M^-1 * J^T * lambda 
			// 
			// Substitute this expression into the velocity constraint: 
			// 
			// J * (v1 + M^-1 * J^T * lambda) + softness * lambda + b = 0 
			// 
			// Now collect coefficients of lambda: 
			// 
			// (J * M^-1 * J^T + softness) * lambda = - J * v1 - b 
			// 
			// Now we define: 
			// 
			// K = J * M^-1 * J^T + softness 
			// 
			// So our new effective mass is K^-1 
			outEffectiveMass = decimal(1.0f) / (inInvEffectiveMass + mSoftness);
		}
		else
		{
			mSoftness = decimal(0.0f);
			mBias = inBias;
		}
	}

	/// Returns if this spring is active
	inline bool					IsActive() const
	{
		return mSoftness != decimal(0.0f);
	}

	/// Get total bias b, including supplied bias and bias for spring: lambda = J v + b
	inline decimal				GetBias(decimal inTotalLambda) const
	{
		// Remainder of post by Erin Catto: http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=4&t=1354
		//
		// Each iteration we are not computing the whole impulse, we are computing an increment to the impulse and we are updating the velocity. 
		// Also, as we solve each constraint we get a perfect v2, but then some other constraint will come along and mess it up. 
		// So we want to patch up the constraint while acknowledging the accumulated impulse and the damaged velocity. 
		// To help with that we use P for the accumulated impulse and lambda as the update. Mathematically we have: 
		// 
		// M * (v2new - v2damaged) = J^T * lambda 
		// J * v2new + softness * (total_lambda + lambda) + b = 0 
		// 
		// If we solve this we get: 
		// 
		// v2new = v2damaged + M^-1 * J^T * lambda 
		// J * (v2damaged + M^-1 * J^T * lambda) + softness * total_lambda + softness * lambda + b = 0 
		// 
		// (J * M^-1 * J^T + softness) * lambda = -(J * v2damaged + softness * total_lambda + b) 
		// 
		// So our lagrange multiplier becomes:
		//
		// lambda = -K^-1 (J v + softness * total_lambda + b)
		//
		// So we return the bias: softness * total_lambda + b
		return mSoftness * inTotalLambda + mBias;
	}
	
private:
	decimal						mBias  = decimal(0.0f);
	decimal						mSoftness  = decimal(0.0f);
};

JPH_NAMESPACE_END
