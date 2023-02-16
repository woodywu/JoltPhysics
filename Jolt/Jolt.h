// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

// Project includes
#include <3rdParty/fpm/include/fpm/fixed.hpp>
#include <3rdParty/fpm/include/fpm/math.hpp>
#include <3rdParty/fpm/include/fpm/ios.hpp>

using decimal = fpm::fixed_24_8;
constexpr decimal decimal_zero = decimal{ 0 };
constexpr decimal decimal_one = decimal{ 1 };

#include <Jolt/Core/Core.h>
#include <Jolt/Core/ARMNeon.h>
#include <Jolt/Core/Memory.h>
#include <Jolt/Core/STLAllocator.h>
#include <Jolt/Core/IssueReporting.h>
#include <Jolt/Math/Math.h>
#include <Jolt/Math/Vec4.h>
#include <Jolt/Math/Mat44.h>
#include <Jolt/Math/Real.h>
