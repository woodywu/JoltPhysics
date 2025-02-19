// SPDX-FileCopyrightText: 2022 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

JPH_NAMESPACE_BEGIN

/// Dynamic resizable matrix class
class [[nodiscard]] DynMatrix
{
public:
	/// Constructor
					DynMatrix(const DynMatrix &) = default;
					DynMatrix(uint inRows, uint inCols)			: mRows(inRows), mCols(inCols) { mElements.resize(inRows * inCols); }

	/// Access an element
	decimal			operator () (uint inRow, uint inCol) const	{ JPH_ASSERT(inRow < mRows && inCol < mCols); return mElements[inRow * mCols + inCol]; }
	decimal &			operator () (uint inRow, uint inCol)		{ JPH_ASSERT(inRow < mRows && inCol < mCols); return mElements[inRow * mCols + inCol]; }

	/// Get dimensions
	uint			GetCols() const								{ return mCols; }
	uint			GetRows() const								{ return mRows; }

private:
	uint			mRows;
	uint			mCols;
	Array<decimal>	mElements;
};

JPH_NAMESPACE_END
