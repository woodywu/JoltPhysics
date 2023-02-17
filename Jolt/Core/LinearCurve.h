// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Jolt/ObjectStream/SerializableObject.h>
#include <Jolt/Core/QuickSort.h>

JPH_NAMESPACE_BEGIN

class StreamOut;
class StreamIn;

// A set of points (x, y) that form a linear curve
class LinearCurve
{
public:
	JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL(LinearCurve)

	/// A point on the curve
	class Point
	{
	public:
		JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL(Point)

		decimal			mX = C0;
		decimal			mY = C0;
	};

	/// Remove all points
	void				Clear()											{ mPoints.clear(); }

	/// Reserve memory for inNumPoints points
	void				Reserve(uint inNumPoints)						{ mPoints.reserve(inNumPoints); }

	/// Add a point to the curve. Points must be inserted in ascending X or Sort() needs to be called when all points have been added.
	/// @param inX X value
	/// @param inY Y value
	void				AddPoint(decimal inX, decimal inY)					{ mPoints.push_back({ inX, inY }); }

	/// Sort the points on X ascending
	void				Sort()											{ QuickSort(mPoints.begin(), mPoints.end(), [](const Point &inLHS, const Point &inRHS) { return inLHS.mX < inRHS.mX; }); }

	/// Get the lowest X value
	decimal				GetMinX() const									{ return mPoints.empty()? C0 : mPoints.front().mX; }

	/// Get the highest X value
	decimal				GetMaxX() const									{ return mPoints.empty()? C0 : mPoints.back().mX; }

	/// Sample value on the curve
	/// @param inX X value to sample at
	/// @return Interpolated Y value
	decimal				GetValue(decimal inX) const;

	/// Saves the state of this object in binary form to inStream.
	void				SaveBinaryState(StreamOut &inStream) const;

	/// Restore the state of this object from inStream.
	void				RestoreBinaryState(StreamIn &inStream);

	/// The points on the curve, should be sorted ascending by x
	using Points = Array<Point>;
	Points				mPoints;
};

JPH_NAMESPACE_END
