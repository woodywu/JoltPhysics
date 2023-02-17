// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

JPH_NAMESPACE_BEGIN

Quat Quat::operator * (QuatArg inRHS) const
{ 
	decimal lx = mValue.GetX();
	decimal ly = mValue.GetY();
	decimal lz = mValue.GetZ();
	decimal lw = mValue.GetW();

	decimal rx = inRHS.mValue.GetX();
	decimal ry = inRHS.mValue.GetY();
	decimal rz = inRHS.mValue.GetZ();
	decimal rw = inRHS.mValue.GetW();

	decimal x = lw * rx + lx * rw + ly * rz - lz * ry;
	decimal y = lw * ry - lx * rz + ly * rw + lz * rx;
	decimal z = lw * rz + lx * ry - ly * rx + lz * rw;
	decimal w = lw * rw - lx * rx - ly * ry - lz * rz;

	return Quat(x, y, z, w);
}

Quat Quat::sRotation(Vec3Arg inAxis, decimal inAngle)
{
    // returns [inAxis * sin(0.5f * inAngle), cos(0.5f * inAngle)]
	JPH_ASSERT(inAxis.IsNormalized());
	Vec4 s, c;
	Vec4::sReplicate(C0P5 * inAngle).SinCos(s, c);
    return Quat(Vec4::sSelect(Vec4(inAxis) * s, c, UVec4(0, 0, 0, 0xffffffffU)));
}

void Quat::GetAxisAngle(Vec3 &outAxis, decimal &outAngle) const
{
	JPH_ASSERT(IsNormalized());
	Quat w_pos = EnsureWPositive();
	decimal abs_w = w_pos.GetW();
	if (abs_w >= C1)
	{ 
		outAxis = Vec3::sZero();
		outAngle = C0;
	}
	else
	{
		outAngle = C2 * ACos(abs_w);
		outAxis = w_pos.GetXYZ().NormalizedOr(Vec3::sZero());
	}
}

Quat Quat::sFromTo(Vec3Arg inFrom, Vec3Arg inTo)
{
	/* 
		Uses (inFrom = v1, inTo = v2): 

		angle = arcos(v1 . v2 / |v1||v2|)
		axis = normalize(v1 x v2)

		Quaternion is then:

		s = sin(angle / 2) 
		x = axis.x * s 
		y = axis.y * s 
		z = axis.z * s 
		w = cos(angle / 2)

		Using identities:

		sin(2 * a) = 2 * sin(a) * cos(a)
		cos(2 * a) = cos(a)^2 - sin(a)^2
		sin(a)^2 + cos(a)^2 = 1

		This reduces to:

		x = (v1 x v2).x
		y = (v1 x v2).y
		z = (v1 x v2).z
		w = |v1||v2| + v1 . v2

		which then needs to be normalized because the whole equation was multiplied by 2 cos(angle / 2)
	*/

	decimal len_v1_v2 = sqrt(inFrom.LengthSq() * inTo.LengthSq());
	decimal w = len_v1_v2 + inFrom.Dot(inTo);

	if (w == C0)
	{
		if (len_v1_v2 == C0)
		{
			// If either of the vectors has zero length, there is no rotation and we return identity
			return Quat::sIdentity();
		}
		else
		{
			// If vectors are perpendicular, take one of the many 180 degree rotations that exist	
			return Quat(Vec4(inFrom.GetNormalizedPerpendicular(), C0));
		}
	}

	Vec3 v = inFrom.Cross(inTo);
	return Quat(Vec4(v, w)).Normalized();
}

template <class Random>
Quat Quat::sRandom(Random &inRandom)
{
	std::uniform_real_distribution<decimal> zero_to_one(0.0f, 1.0f);
	decimal x0 = zero_to_one(inRandom);
	decimal r1 = sqrt(1.0f - x0), r2 = sqrt(x0);
	std::uniform_real_distribution<decimal> zero_to_two_pi(0.0f, 2.0f * JPH_PI);
	Vec4 s, c;
	Vec4(zero_to_two_pi(inRandom), zero_to_two_pi(inRandom), 0, 0).SinCos(s, c);
	return Quat(s.GetX() * r1, c.GetX() * r1, s.GetY() * r2, c.GetY() * r2);
}

Quat Quat::sEulerAngles(Vec3Arg inAngles)
{
	Vec4 half(C0P5 * inAngles);
	Vec4 s, c;
	half.SinCos(s, c);

	decimal cx = c.GetX();
	decimal sx = s.GetX();
	decimal cy = c.GetY();
	decimal sy = s.GetY();
	decimal cz = c.GetZ();
	decimal sz = s.GetZ();

	return Quat(
		cz * sx * cy - sz * cx * sy,
		cz * cx * sy + sz * sx * cy,
		sz * cx * cy - cz * sx * sy,
		cz * cx * cy + sz * sx * sy);
}

Vec3 Quat::GetEulerAngles() const
{
	decimal y_sq = GetY() * GetY();

	// X
	decimal t0 = C2 * (GetW() * GetX() + GetY() * GetZ());
	decimal t1 = C1 - C2 * (GetX() * GetX() + y_sq);

	// Y
	decimal t2 = C2 * (GetW() * GetY() - GetZ() * GetX());
	t2 = t2 > C1? C1 : t2;
	t2 = t2 < -C1? -C1 : t2;

	// Z
	decimal t3 = C2 * (GetW() * GetZ() + GetX() * GetY());
	decimal t4 = C1 - C2 * (y_sq + GetZ() * GetZ());  

	return Vec3(ATan2(t0, t1), ASin(t2), ATan2(t3, t4));
}

Quat Quat::GetTwist(Vec3Arg inAxis) const
{ 
	Quat twist(Vec4(GetXYZ().Dot(inAxis) * inAxis, GetW()));
	decimal twist_len = twist.LengthSq();
	if (twist_len != C0)
		return twist / sqrt(twist_len);
	else
		return Quat::sIdentity();
}

void Quat::GetSwingTwist(Quat &outSwing, Quat &outTwist) const
{
	decimal x = GetX(), y = GetY(), z = GetZ(), w = GetW();
	decimal s = sqrt(Square(w) + Square(x));
	if (s != C0)
	{
		outTwist = Quat(x / s, C0, C0, w / s);
		outSwing = Quat(C0, (w * y - x * z) / s, (w * z + x * y) / s, s);
	}
	else
	{
		// If both x and w are zero, this must be a 180 degree rotation around either y or z
		outTwist = Quat::sIdentity();
		outSwing = *this;
	}
}

Quat Quat::LERP(QuatArg inDestination, decimal inFraction) const
{
	decimal scale0 = C1 - inFraction;
	return Quat(Vec4::sReplicate(scale0) * mValue + Vec4::sReplicate(inFraction) * inDestination.mValue);
}

Quat Quat::SLERP(QuatArg inDestination, decimal inFraction) const
{	
    // Difference at which to LERP instead of SLERP
	const decimal delta = decimal(0.0001f);

	// Calc cosine
	decimal sign_scale1 = C1;
    decimal cos_omega = Dot(inDestination);
	
	// Adjust signs (if necessary)
	if (cos_omega < C0)
	{
		cos_omega = -cos_omega;
		sign_scale1 = -C1;
	}
	
	// Calculate coefficients
	decimal scale0, scale1;
	if (C1 - cos_omega > delta) 
	{
		// Standard case (slerp)
		decimal omega = ACos(cos_omega);
		decimal sin_omega = Sin(omega);
		scale0 = Sin((C1 - inFraction) * omega) / sin_omega;
		scale1 = sign_scale1 * Sin(inFraction * omega) / sin_omega;
	} 
	else 
	{        
		// Quaternions are very close so we can do a linear interpolation
		scale0 = C1 - inFraction;
		scale1 = sign_scale1 * inFraction;
	}

	// Interpolate between the two quaternions
	return Quat(Vec4::sReplicate(scale0) * mValue + Vec4::sReplicate(scale1) * inDestination.mValue).Normalized();
}

Vec3 Quat::operator * (Vec3Arg inValue) const
{
	// Rotating a vector by a quaternion is done by: p' = q * p * q^-1 (q^-1 = conjugated(q) for a unit quaternion)
	JPH_ASSERT(IsNormalized());
	return Vec3((*this * Quat(Vec4(inValue, C0)) * Conjugated()).mValue);
}

Vec3 Quat::InverseRotate(Vec3Arg inValue) const
{
	JPH_ASSERT(IsNormalized());
	return Vec3((Conjugated() * Quat(Vec4(inValue, C0)) * *this).mValue);
}

Vec3 Quat::RotateAxisX() const												
{ 
	// This is *this * Vec3::sAxisX() written out:
	JPH_ASSERT(IsNormalized());
	decimal x = GetX(), y = GetY(), z = GetZ(), w = GetW();
	decimal tx = C2 * x, tw = C2 * w; 
	return Vec3(tx * x + tw * w - C1, tx * y + z * tw, tx * z - y * tw); 
}

Vec3 Quat::RotateAxisY() const												
{ 
	// This is *this * Vec3::sAxisY() written out:
	JPH_ASSERT(IsNormalized());
	decimal x = GetX(), y = GetY(), z = GetZ(), w = GetW();
	decimal ty = C2 * y, tw = C2 * w; 
	return Vec3(x * ty - z * tw, tw * w + ty * y - C1, x * tw + ty * z); 
}

Vec3 Quat::RotateAxisZ() const												
{ 
	// This is *this * Vec3::sAxisZ() written out:
	JPH_ASSERT(IsNormalized());
	decimal x = GetX(), y = GetY(), z = GetZ(), w = GetW();
	decimal tz = C2 * z, tw = C2 * w; 
	return Vec3(x * tz + y * tw, y * tz - x * tw, tw * w + tz * z - C1); 
}

void Quat::StoreFloat3(Float3 *outV) const
{
	JPH_ASSERT(IsNormalized());
	EnsureWPositive().GetXYZ().StoreFloat3(outV);
}

Quat Quat::sLoadFloat3Unsafe(const Float3 &inV)
{
	Vec3 v = Vec3::sLoadFloat3Unsafe(inV);
	decimal w = sqrt(max(C1 - v.LengthSq(), C0)); // It is possible that the length of v is a fraction above 1, and we don't want to introduce NaN's in that case so we clamp to 0
	return Quat(Vec4(v, w));
}

JPH_NAMESPACE_END
