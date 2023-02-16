// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Jolt/Math/Fixed3.h>
#include <Jolt/Math/Swizzle.h>
#include <Jolt/Math/MathTypes.h>

JPH_NAMESPACE_BEGIN

/// 3 component vector (stored as 4 vectors). 
/// Note that we keep the 4th component the same as the 3rd component to avoid divisions by zero when JPH_FLOATING_POINT_EXCEPTIONS_ENABLED defined
class [[nodiscard]] alignas(JPH_VECTOR_ALIGNMENT) FixedVec3
{
public:
	JPH_OVERRIDE_NEW_DELETE

	// Underlying vector type
	using Type = struct { decimal mData[4]; };
	using TypeArg = const Type&;

	// Argument type
	using ArgType = FixedVec3Arg;

	/// Constructor
								FixedVec3() = default; ///< Intentionally not initialized for performance reasons
								FixedVec3(const FixedVec3 &inRHS) = default;
	JPH_INLINE explicit			FixedVec3(Vec3Arg inRHS);
	JPH_INLINE explicit			FixedVec3(Vec4Arg inRHS);
	JPH_INLINE					FixedVec3(Type inRHS) : mValue(inRHS)				{ CheckW(); }

	/// Load 3 floats from memory
	explicit JPH_INLINE			FixedVec3(const Fixed3 &inV);

	/// Create a vector from 3 components
	JPH_INLINE					FixedVec3(decimal inX, decimal inY, decimal inZ);

	/// Vector with all zeros
	static JPH_INLINE FixedVec3		sZero();

	/// Vector with all NaN's
	static JPH_INLINE FixedVec3		sNaN();

	/// Vectors with the principal axis
	static JPH_INLINE FixedVec3		sAxisX()										{ return FixedVec3(decimal_one, decimal_zero, decimal_zero); }
	static JPH_INLINE FixedVec3		sAxisY()										{ return FixedVec3(decimal_zero, decimal_one, decimal_zero); }
	static JPH_INLINE FixedVec3		sAxisZ()										{ return FixedVec3(decimal_zero, decimal_zero, decimal_one); }

	/// Replicate inV across all components
	static JPH_INLINE FixedVec3		sReplicate(decimal inV);
		
	/// Load 3 floats from memory (reads 32 bits extra which it doesn't use)
	static JPH_INLINE FixedVec3		sLoadFixed3Unsafe(const Fixed3 &inV);

	/// Return the minimum value of each of the components
	static JPH_INLINE FixedVec3		sMin(FixedVec3Arg inV1, FixedVec3Arg inV2);

	/// Return the maximum of each of the components
	static JPH_INLINE FixedVec3		sMax(FixedVec3Arg inV1, FixedVec3Arg inV2);

	/// Clamp a vector between min and max (component wise)
	static JPH_INLINE FixedVec3		sClamp(FixedVec3Arg inV, FixedVec3Arg inMin, FixedVec3Arg inMax);

	/// Equals (component wise)
	static JPH_INLINE UVec4		sEquals(FixedVec3Arg inV1, FixedVec3Arg inV2);

	/// Less than (component wise)
	static JPH_INLINE UVec4		sLess(FixedVec3Arg inV1, FixedVec3Arg inV2);

	/// Less than or equal (component wise)
	static JPH_INLINE UVec4		sLessOrEqual(FixedVec3Arg inV1, FixedVec3Arg inV2);

	/// Greater than (component wise)
	static JPH_INLINE UVec4		sGreater(FixedVec3Arg inV1, FixedVec3Arg inV2);

	/// Greater than or equal (component wise)
	static JPH_INLINE UVec4		sGreaterOrEqual(FixedVec3Arg inV1, FixedVec3Arg inV2);

	/// Calculates inMul1 * inMul2 + inAdd
	static JPH_INLINE FixedVec3		sFusedMultiplyAdd(FixedVec3Arg inMul1, FixedVec3Arg inMul2, FixedVec3Arg inAdd);

	/// Component wise select, returns inV1 when highest bit of inControl = 0 and inV2 when highest bit of inControl = 1
	static JPH_INLINE FixedVec3		sSelect(FixedVec3Arg inV1, FixedVec3Arg inV2, UVec4Arg inControl);

	/// Logical or (component wise)
	static JPH_INLINE FixedVec3		sOr(FixedVec3Arg inV1, FixedVec3Arg inV2);

	/// Logical xor (component wise)
	static JPH_INLINE FixedVec3		sXor(FixedVec3Arg inV1, FixedVec3Arg inV2);

	/// Logical and (component wise)
	static JPH_INLINE FixedVec3		sAnd(FixedVec3Arg inV1, FixedVec3Arg inV2);

	/// Get unit vector given spherical coordinates
	/// inTheta \f$\in [0, \pi]\f$ is angle between vector and z-axis
	/// inPhi \f$\in [0, 2 \pi]\f$ is the angle in the xy-plane starting from the x axis and rotating counter clockwise around the z-axis
	static JPH_INLINE FixedVec3		sUnitSpherical(float inTheta, float inPhi);

	/// A set of vectors uniformly spanning the surface of a unit sphere, usable for debug purposes
	static const std::vector<FixedVec3> sUnitSphere;

	/// Get random unit vector
	template <class Random>
	static inline FixedVec3			sRandom(Random &inRandom);

	/// Get individual components

	JPH_INLINE decimal			GetX() const									{ return mF32[0]; }
	JPH_INLINE decimal			GetY() const									{ return mF32[1]; }
	JPH_INLINE decimal			GetZ() const									{ return mF32[2]; }
	
	/// Set individual components
	JPH_INLINE void				SetX(decimal inX)									{ mF32[0] = inX; }
	JPH_INLINE void				SetY(decimal inY)									{ mF32[1] = inY; }
	JPH_INLINE void				SetZ(decimal inZ)									{ mF32[2] = mF32[3] = inZ; } // Assure Z and W are the same

	/// Get float component by index
	JPH_INLINE decimal			operator [] (uint inCoordinate) const			{ JPH_ASSERT(inCoordinate < 3); return mF32[inCoordinate]; }

	/// Set float component by index
	JPH_INLINE void				SetComponent(uint inCoordinate, decimal inValue)	{ JPH_ASSERT(inCoordinate < 3); mF32[inCoordinate] = inValue; mValue = sFixW(mValue); } // Assure Z and W are the same

	/// Comparison
	JPH_INLINE bool				operator == (FixedVec3Arg inV2) const;
	JPH_INLINE bool				operator != (FixedVec3Arg inV2) const				{ return !(*this == inV2); }

	/// Test if two vectors are close
	JPH_INLINE bool				IsClose(FixedVec3Arg inV2, decimal inMaxDistSq = decimal(1.0e-12f)) const;

	/// Test if vector is near zero
	JPH_INLINE bool				IsNearZero(decimal inMaxDistSq = decimal(1.0e-12f)) const;

	/// Test if vector is normalized
	JPH_INLINE bool				IsNormalized(decimal inTolerance = decimal(1.0e-6f)) const;

	/// Test if vector contains NaN elements
	JPH_INLINE bool				IsNaN() const;

	/// Multiply two float vectors (component wise)
	JPH_INLINE FixedVec3				operator * (FixedVec3Arg inV2) const;

	/// Multiply vector with float
	JPH_INLINE FixedVec3				operator * (decimal inV2) const;

	/// Multiply vector with float
	friend JPH_INLINE FixedVec3		operator * (decimal inV1, FixedVec3Arg inV2);

	/// Divide vector by float
	JPH_INLINE FixedVec3				operator / (decimal inV2) const;

	/// Multiply vector with float
	JPH_INLINE FixedVec3 &			operator *= (decimal inV2);

	/// Multiply vector with vector
	JPH_INLINE FixedVec3 &			operator *= (FixedVec3Arg inV2);

	/// Divide vector by float
	JPH_INLINE FixedVec3 &			operator /= (decimal inV2);

	/// Add two float vectors (component wise)
	JPH_INLINE FixedVec3				operator + (FixedVec3Arg inV2) const;

	/// Add two float vectors (component wise)
	JPH_INLINE FixedVec3 &			operator += (FixedVec3Arg inV2);

	/// Negate
	JPH_INLINE FixedVec3				operator - () const;

	/// Subtract two float vectors (component wise)
	JPH_INLINE FixedVec3				operator - (FixedVec3Arg inV2) const;

	/// Add two float vectors (component wise)
	JPH_INLINE FixedVec3 &			operator -= (FixedVec3Arg inV2);

	/// Divide (component wise)
	JPH_INLINE FixedVec3				operator / (FixedVec3Arg inV2) const;

	/// Swizzle the elements in inV
	template<uint32 SwizzleX, uint32 SwizzleY, uint32 SwizzleZ>
	JPH_INLINE FixedVec3				Swizzle() const;

	/// Replicate the X component to all components
	JPH_INLINE Vec4				SplatX() const;

	/// Replicate the Y component to all components
	JPH_INLINE Vec4				SplatY() const;

	/// Replicate the Z component to all components
	JPH_INLINE Vec4				SplatZ() const;

	/// Get index of component with lowest value
	JPH_INLINE int				GetLowestComponentIndex() const;

	/// Get index of component with highest value
	JPH_INLINE int				GetHighestComponentIndex() const;

	/// Return the absolute value of each of the components
	JPH_INLINE FixedVec3				Abs() const;

	/// Reciprocal vector (1 / value) for each of the components
	JPH_INLINE FixedVec3				Reciprocal() const;

	/// Cross product
	JPH_INLINE FixedVec3				Cross(FixedVec3Arg inV2) const;

	/// Dot product, returns the dot product in X, Y and Z components
	JPH_INLINE FixedVec3				DotV(FixedVec3Arg inV2) const;

	/// Dot product, returns the dot product in X, Y, Z and W components
	JPH_INLINE Vec4				DotV4(FixedVec3Arg inV2) const;

	/// Dot product
	JPH_INLINE decimal			Dot(FixedVec3Arg inV2) const;

	/// Squared length of vector
	JPH_INLINE decimal			LengthSq() const;

	/// Length of vector
	JPH_INLINE decimal			Length() const;

	/// Normalize vector
	JPH_INLINE FixedVec3				Normalized() const;

	/// Normalize vector or return inZeroValue if the length of the vector is zero
	JPH_INLINE FixedVec3				NormalizedOr(FixedVec3Arg inZeroValue) const;

	/// Store 3 floats to memory
	JPH_INLINE void				StoreFixed3(Fixed3 *outV) const;

	/// Convert each component from a float to an int
	JPH_INLINE UVec4			ToInt() const;

	/// Reinterpret FixedVec3 as a UVec4 (doesn't change the bits)
	JPH_INLINE UVec4			ReinterpretAsInt() const;

	/// Get the minimum of X, Y and Z
	JPH_INLINE decimal			ReduceMin() const;

	/// Get the maximum of X, Y and Z
	JPH_INLINE decimal			ReduceMax() const;

	/// Component wise square root
	JPH_INLINE FixedVec3				Sqrt() const;

	/// Get normalized vector that is perpendicular to this vector
	JPH_INLINE FixedVec3				GetNormalizedPerpendicular() const;

	/// Get vector that contains the sign of each element (returns 1.0f if positive, -1.0f if negative)
	JPH_INLINE FixedVec3				GetSign() const;

	/// To String
	friend ostream &			operator << (ostream &inStream, FixedVec3Arg inV)
	{
		inStream << inV.mF32[0] << ", " << inV.mF32[1] << ", " << inV.mF32[2];
		return inStream;
	}

	/// Internal helper function that checks that W is equal to Z, so e.g. dividing by it should not generate div by 0
	JPH_INLINE void				CheckW() const;
	
	/// Internal helper function that ensures that the Z component is replicated to the W component to prevent divisions by zero
	static JPH_INLINE Type		sFixW(Type inValue);

	union
	{
		Type					mValue;
		decimal					mF32[4];
	};
};

static_assert(is_trivial<FixedVec3>(), "Is supposed to be a trivial type!");

JPH_NAMESPACE_END

#include "FixedVec3.inl"
