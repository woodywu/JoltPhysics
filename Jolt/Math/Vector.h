// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

JPH_NAMESPACE_BEGIN

/// Templatized vector class
template <uint Rows>
class [[nodiscard]] Vector
{
public:
	/// Constructor
	inline						Vector() = default;
	inline						Vector(const Vector &inRHS)								{ *this = inRHS; }

	/// Dimensions
	inline uint					GetRows() const											{ return Rows; }

	/// Vector with all zeros
	inline void					SetZero()
	{
		for (uint r = 0; r < Rows; ++r)
			mF32[r] = C0;
	}

	inline static Vector		sZero()													{ Vector v; v.SetZero(); return v; }

	/// Copy a (part) of another vector into this vector
	template <class OtherVector>
		void					CopyPart(const OtherVector &inV, uint inSourceRow, uint inNumRows, uint inDestRow)
		{
			for (uint r = 0; r < inNumRows; ++r)
				mF32[inDestRow + r] = inV[inSourceRow + r];
		}

	/// Get decimal component by index
	inline decimal				operator [] (uint inCoordinate) const			
	{ 
		JPH_ASSERT(inCoordinate < Rows); 
		return mF32[inCoordinate]; 
	}
	
	inline decimal &				operator [] (uint inCoordinate)
	{ 
		JPH_ASSERT(inCoordinate < Rows); 
		return mF32[inCoordinate]; 
	}

	/// Comparison
	inline bool					operator == (const Vector &inV2) const
	{
		for (uint r = 0; r < Rows; ++r)
			if (mF32[r] != inV2.mF32[r])
				return false;
		return true;
	}

	inline bool					operator != (const Vector &inV2) const
	{
		for (uint r = 0; r < Rows; ++r)
			if (mF32[r] != inV2.mF32[r])
				return true;
		return false;
	}

	/// Test if vector consists of all zeros
	inline bool					IsZero() const
	{
		for (uint r = 0; r < Rows; ++r)
			if (mF32[r] != C0)
				return false;
		return true;
	}

	/// Test if two vectors are close to each other
	inline bool					IsClose(const Vector &inV2, decimal inMaxDistSq = decimal(1.0e-12f))
	{
		return (inV2 - *this).LengthSq() <= inMaxDistSq;
	}

	/// Assignment
	inline Vector &				operator = (const Vector &inV2)
	{
		for (uint r = 0; r < Rows; ++r)
			mF32[r] = inV2.mF32[r];
		return *this;
	}

	/// Multiply vector with decimal
	inline Vector				operator * (const decimal inV2) const
	{
		Vector v;
		for (uint r = 0; r < Rows; ++r)
			v.mF32[r] = mF32[r] * inV2;
		return v;
	}

	inline Vector &				operator *= (const decimal inV2) 
	{
		for (uint r = 0; r < Rows; ++r)
			mF32[r] *= inV2;
		return *this;
	}

	/// Multiply vector with decimal
	inline friend Vector		operator * (const decimal inV1, const Vector &inV2)
	{
		return inV2 * inV1;
	}

	/// Divide vector by decimal
	inline Vector				operator / (decimal inV2) const
	{
		Vector v;
		for (uint r = 0; r < Rows; ++r)
			v.mF32[r] = mF32[r] / inV2;
		return v;
	}

	/// Add two decimal vectors (component wise)
	inline Vector				operator + (const Vector &inV2) const
	{
		Vector v;
		for (uint r = 0; r < Rows; ++r)
			v.mF32[r] = mF32[r] + inV2.mF32[r];
		return v;
	}

	inline Vector &				operator += (const Vector &inV2)
	{
		for (uint r = 0; r < Rows; ++r)
			mF32[r] += inV2.mF32[r];
		return *this;
	}

	/// Negate
	inline Vector				operator - () const
	{
		Vector v;
		for (uint r = 0; r < Rows; ++r)
			v.mF32[r] = -mF32[r];
		return v;
	}

	/// Subtract two decimal vectors (component wise)
	inline Vector				operator - (const Vector &inV2) const
	{
		Vector v;
		for (uint r = 0; r < Rows; ++r)
			v.mF32[r] = mF32[r] - inV2.mF32[r];
		return v;
	}

	inline Vector &				operator -= (const Vector &inV2) 
	{
		for (uint r = 0; r < Rows; ++r)
			mF32[r] -= inV2.mF32[r];
		return *this;
	}

	/// Dot product
	inline decimal				Dot(const Vector &inV2) const
	{
		decimal dot = C0;
		for (uint r = 0; r < Rows; ++r)
			dot += mF32[r] * inV2.mF32[r];
		return dot;
	}

	/// Squared length of vector
	inline decimal				LengthSq() const
	{
		return Dot(*this);
	}

	/// Length of vector
	inline decimal				Length() const
	{
		return sqrt(LengthSq());
	}

	/// Check if vector is normalized
	inline bool					IsNormalized(decimal inToleranceSq = decimal(1.0e-6f))
	{
		return abs(LengthSq() - C1) <= inToleranceSq;
	}

	/// Normalize vector
	inline Vector				Normalized() const
	{
		return *this / Length();
	}

	/// To String
	friend ostream &			operator << (ostream &inStream, const Vector &inV)
	{
		inStream << "[";
		for (uint i = 0; i < Rows - 1; ++i)
			inStream << inV.mF32[i] << ", ";
		inStream << inV.mF32[Rows - 1] << "]";
		return inStream;
	}

	decimal						mF32[Rows];
};

JPH_NAMESPACE_END
