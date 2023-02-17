// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

JPH_NAMESPACE_BEGIN

UVec4::UVec4(uint32 inX, uint32 inY, uint32 inZ, uint32 inW)
{
	mU32[0] = inX;
	mU32[1] = inY;
	mU32[2] = inZ;
	mU32[3] = inW;
}

bool UVec4::operator == (UVec4Arg inV2) const
{
	return sEquals(*this, inV2).TestAllTrue();
}

template<uint32 SwizzleX, uint32 SwizzleY, uint32 SwizzleZ, uint32 SwizzleW>
UVec4 UVec4::Swizzle() const
{
	static_assert(SwizzleX <= 3, "SwizzleX template parameter out of range");
	static_assert(SwizzleY <= 3, "SwizzleY template parameter out of range");
	static_assert(SwizzleZ <= 3, "SwizzleZ template parameter out of range");
	static_assert(SwizzleW <= 3, "SwizzleW template parameter out of range");

	return UVec4(mU32[SwizzleX], mU32[SwizzleY], mU32[SwizzleZ], mU32[SwizzleW]);
}

UVec4 UVec4::sZero()
{
	return UVec4(0, 0, 0, 0);
}

UVec4 UVec4::sReplicate(uint32 inV)
{
	return UVec4(inV, inV, inV, inV);
}

UVec4 UVec4::sLoadInt(const uint32 *inV)
{
	return UVec4(*inV, 0, 0, 0);
}

UVec4 UVec4::sLoadInt4(const uint32 *inV)
{
	return UVec4(inV[0], inV[1], inV[2], inV[3]);
}

UVec4 UVec4::sLoadInt4Aligned(const uint32 *inV)
{
	return UVec4(inV[0], inV[1], inV[2], inV[3]);
}

template <const int Scale>
UVec4 UVec4::sGatherInt4(const uint32 *inBase, UVec4Arg inOffsets)
{
	return Vec4::sGatherFloat4<Scale>(reinterpret_cast<const decimal *>(inBase), inOffsets).ReinterpretAsInt();
}

UVec4 UVec4::sMin(UVec4Arg inV1, UVec4Arg inV2)
{
	UVec4 result;
	for (int i = 0; i < 4; i++)
		result.mU32[i] = min(inV1.mU32[i], inV2.mU32[i]);
	return result;
}

UVec4 UVec4::sMax(UVec4Arg inV1, UVec4Arg inV2)
{
	UVec4 result;
	for (int i = 0; i < 4; i++)
		result.mU32[i] = max(inV1.mU32[i], inV2.mU32[i]);
	return result;
}

UVec4 UVec4::sEquals(UVec4Arg inV1, UVec4Arg inV2)
{
	return UVec4(inV1.mU32[0] == inV2.mU32[0]? 0xffffffffu : 0, 
				 inV1.mU32[1] == inV2.mU32[1]? 0xffffffffu : 0, 
				 inV1.mU32[2] == inV2.mU32[2]? 0xffffffffu : 0, 
				 inV1.mU32[3] == inV2.mU32[3]? 0xffffffffu : 0);
}

UVec4 UVec4::sSelect(UVec4Arg inV1, UVec4Arg inV2, UVec4Arg inControl)
{
	UVec4 result;
	for (int i = 0; i < 4; i++)
		result.mU32[i] = inControl.mU32[i] ? inV2.mU32[i] : inV1.mU32[i];
	return result;
}

UVec4 UVec4::sOr(UVec4Arg inV1, UVec4Arg inV2)
{
	return UVec4(inV1.mU32[0] | inV2.mU32[0], 
				 inV1.mU32[1] | inV2.mU32[1], 
				 inV1.mU32[2] | inV2.mU32[2], 
				 inV1.mU32[3] | inV2.mU32[3]);
}

UVec4 UVec4::sXor(UVec4Arg inV1, UVec4Arg inV2)
{
	return UVec4(inV1.mU32[0] ^ inV2.mU32[0], 
				 inV1.mU32[1] ^ inV2.mU32[1], 
				 inV1.mU32[2] ^ inV2.mU32[2], 
				 inV1.mU32[3] ^ inV2.mU32[3]);
}

UVec4 UVec4::sAnd(UVec4Arg inV1, UVec4Arg inV2)
{
	return UVec4(inV1.mU32[0] & inV2.mU32[0], 
				 inV1.mU32[1] & inV2.mU32[1], 
				 inV1.mU32[2] & inV2.mU32[2], 
				 inV1.mU32[3] & inV2.mU32[3]);
}


UVec4 UVec4::sNot(UVec4Arg inV1)
{
	return UVec4(~inV1.mU32[0], ~inV1.mU32[1], ~inV1.mU32[2], ~inV1.mU32[3]);
}

UVec4 UVec4::sSort4True(UVec4Arg inValue, UVec4Arg inIndex)
{
	// If inValue.z is false then shift W to Z
	UVec4 v = UVec4::sSelect(inIndex.Swizzle<SWIZZLE_X, SWIZZLE_Y, SWIZZLE_W, SWIZZLE_W>(), inIndex, inValue.SplatZ());

	// If inValue.y is false then shift Z and further to Y and further
	v = UVec4::sSelect(v.Swizzle<SWIZZLE_X, SWIZZLE_Z, SWIZZLE_W, SWIZZLE_W>(), v, inValue.SplatY());

	// If inValue.x is false then shift X and furhter to Y and furhter
	v = UVec4::sSelect(v.Swizzle<SWIZZLE_Y, SWIZZLE_Z, SWIZZLE_W, SWIZZLE_W>(), v, inValue.SplatX());

	return v;
}

UVec4 UVec4::operator * (UVec4Arg inV2) const
{
	UVec4 result;
	for (int i = 0; i < 4; i++)
		result.mU32[i] = mU32[i] * inV2.mU32[i];
	return result;
}

UVec4 UVec4::operator + (UVec4Arg inV2)
{
	return UVec4(mU32[0] + inV2.mU32[0], 
				 mU32[1] + inV2.mU32[1], 
				 mU32[2] + inV2.mU32[2], 
				 mU32[3] + inV2.mU32[3]);
}

UVec4 &UVec4::operator += (UVec4Arg inV2)
{
	for (int i = 0; i < 4; ++i)
		mU32[i] += inV2.mU32[i];
	return *this;
}

UVec4 UVec4::SplatX() const
{
	return UVec4(mU32[0], mU32[0], mU32[0], mU32[0]);
}

UVec4 UVec4::SplatY() const
{
	return UVec4(mU32[1], mU32[1], mU32[1], mU32[1]);
}

UVec4 UVec4::SplatZ() const
{
	return UVec4(mU32[2], mU32[2], mU32[2], mU32[2]);
}

UVec4 UVec4::SplatW() const
{
	return UVec4(mU32[3], mU32[3], mU32[3], mU32[3]);
}

Vec4 UVec4::ToFloat() const
{
	return Vec4(decimal(mU32[0]), decimal(mU32[1]), decimal(mU32[2]), decimal(mU32[3]));
}

Vec4 UVec4::ReinterpretAsFloat() const
{
	return *reinterpret_cast<const Vec4 *>(this);
}

void UVec4::StoreInt4(uint32 *outV) const
{
	for (int i = 0; i < 4; ++i)
		outV[i] = mU32[i];
}

void UVec4::StoreInt4Aligned(uint32 *outV) const
{
	for (int i = 0; i < 4; ++i)
		outV[i] = mU32[i];
}

int UVec4::CountTrues() const
{
	return (mU32[0] >> 31) + (mU32[1] >> 31) + (mU32[2] >> 31) + (mU32[3] >> 31);
}

int UVec4::GetTrues() const
{
	return (mU32[0] >> 31) | ((mU32[1] >> 31) << 1) | ((mU32[2] >> 31) << 2) | ((mU32[3] >> 31) << 3);
}

bool UVec4::TestAnyTrue() const
{
	return GetTrues() != 0;
}

bool UVec4::TestAnyXYZTrue() const
{
	return (GetTrues() & 0b111) != 0;
}

bool UVec4::TestAllTrue() const
{
	return GetTrues() == 0b1111;
}

bool UVec4::TestAllXYZTrue() const
{
	return (GetTrues() & 0b111) == 0b111;
}

template <const uint Count>
UVec4 UVec4::LogicalShiftLeft() const
{
	static_assert(Count <= 31, "Invalid shift");

	return UVec4(mU32[0] << Count, mU32[1] << Count, mU32[2] << Count, mU32[3] << Count);
}

template <const uint Count>
UVec4 UVec4::LogicalShiftRight() const
{
	static_assert(Count <= 31, "Invalid shift");

	return UVec4(mU32[0] >> Count, mU32[1] >> Count, mU32[2] >> Count, mU32[3] >> Count);
}

template <const uint Count>
UVec4 UVec4::ArithmeticShiftRight() const
{
	static_assert(Count <= 31, "Invalid shift");

	return UVec4(uint32(int32_t(mU32[0]) >> Count), 
				 uint32(int32_t(mU32[1]) >> Count), 
				 uint32(int32_t(mU32[2]) >> Count), 
				 uint32(int32_t(mU32[3]) >> Count));
}

UVec4 UVec4::Expand4Uint16Lo() const
{
	return UVec4(mU32[0] & 0xffff, 
				 (mU32[0] >> 16) & 0xffff, 
				 mU32[1] & 0xffff, 
				 (mU32[1] >> 16) & 0xffff);
}

UVec4 UVec4::Expand4Uint16Hi() const
{
	return UVec4(mU32[2] & 0xffff, 
				 (mU32[2] >> 16) & 0xffff, 
				 mU32[3] & 0xffff, 
				 (mU32[3] >> 16) & 0xffff);
}

UVec4 UVec4::Expand4Byte0() const
{
	UVec4 result;
	for (int i = 0; i < 4; i++)
		result.mU32[i] = (mU32[0] >> (i * 8)) & 0xff;
	return result;
}

UVec4 UVec4::Expand4Byte4() const
{
	UVec4 result;
	for (int i = 0; i < 4; i++)
		result.mU32[i] = (mU32[1] >> (i * 8)) & 0xff;
	return result;
}

UVec4 UVec4::Expand4Byte8() const
{
	UVec4 result;
	for (int i = 0; i < 4; i++)
		result.mU32[i] = (mU32[2] >> (i * 8)) & 0xff;
	return result;
}

UVec4 UVec4::Expand4Byte12() const
{
	UVec4 result;
	for (int i = 0; i < 4; i++)
		result.mU32[i] = (mU32[3] >> (i * 8)) & 0xff;
	return result;
}

UVec4 UVec4::ShiftComponents4Minus(int inCount) const
{
	UVec4 result = UVec4::sZero();
	for (int i = 0; i < inCount; i++)
		result.mU32[i] = mU32[i + 4 - inCount];
	return result;
}

JPH_NAMESPACE_END
