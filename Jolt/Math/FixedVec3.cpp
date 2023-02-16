// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Jolt/Jolt.h>

#include <Jolt/Core/UnorderedSet.h>
#include <Jolt/Math/FixedVec3.h>

JPH_NAMESPACE_BEGIN

static void sCreateVertices(std::unordered_set<FixedVec3> &ioVertices, FixedVec3Arg inDir1, FixedVec3Arg inDir2, FixedVec3Arg inDir3, int inLevel)
{
	FixedVec3 center1 = (inDir1 + inDir2).Normalized();
	FixedVec3 center2 = (inDir2 + inDir3).Normalized();
	FixedVec3 center3 = (inDir3 + inDir1).Normalized();

	ioVertices.insert(center1);
	ioVertices.insert(center2);
	ioVertices.insert(center3);

	if (inLevel > 0)
	{
		int new_level = inLevel - 1;
		sCreateVertices(ioVertices, inDir1, center1, center3, new_level);
		sCreateVertices(ioVertices, center1, center2, center3, new_level);
		sCreateVertices(ioVertices, center1, inDir2, center2, new_level);
		sCreateVertices(ioVertices, center3, center2, inDir3, new_level);
	}
}

const std::vector<FixedVec3> FixedVec3::sUnitSphere = []() { 

	const int level = 3;

	std::unordered_set<FixedVec3> verts;
	
	// Add unit axis
	verts.insert(FixedVec3::sAxisX());
	verts.insert(-FixedVec3::sAxisX());
	verts.insert(FixedVec3::sAxisY());
	verts.insert(-FixedVec3::sAxisY());
	verts.insert(FixedVec3::sAxisZ());
	verts.insert(-FixedVec3::sAxisZ());

	// Subdivide
	sCreateVertices(verts, FixedVec3::sAxisX(), FixedVec3::sAxisY(), FixedVec3::sAxisZ(), level);
	sCreateVertices(verts, -FixedVec3::sAxisX(), FixedVec3::sAxisY(), FixedVec3::sAxisZ(), level);
	sCreateVertices(verts, FixedVec3::sAxisX(), -FixedVec3::sAxisY(), FixedVec3::sAxisZ(), level);
	sCreateVertices(verts, -FixedVec3::sAxisX(), -FixedVec3::sAxisY(), FixedVec3::sAxisZ(), level);
	sCreateVertices(verts, FixedVec3::sAxisX(), FixedVec3::sAxisY(), -FixedVec3::sAxisZ(), level);
	sCreateVertices(verts, -FixedVec3::sAxisX(), FixedVec3::sAxisY(), -FixedVec3::sAxisZ(), level);
	sCreateVertices(verts, FixedVec3::sAxisX(), -FixedVec3::sAxisY(), -FixedVec3::sAxisZ(), level);
	sCreateVertices(verts, -FixedVec3::sAxisX(), -FixedVec3::sAxisY(), -FixedVec3::sAxisZ(), level);

	return std::vector<FixedVec3>(verts.begin(), verts.end());
}();

JPH_NAMESPACE_END
