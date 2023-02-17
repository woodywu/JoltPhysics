// SPDX-FileCopyrightText: 2022 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <3rdParty/fpm/include/fpm/fixed.hpp>
#include <3rdParty/fpm/include/fpm/math.hpp>
#include <3rdParty/fpm/include/fpm/ios.hpp>

using decimal = fpm::fixed_24_8;

constexpr decimal C0 = decimal(0);
constexpr decimal C0P5 = decimal(0.5);
constexpr decimal C0P1 = decimal(0.1);
constexpr decimal C1 = decimal(1);
constexpr decimal C2 = decimal(2);
constexpr decimal C3 = decimal(3);

constexpr decimal FIX_MIN = std::numeric_limits<decimal>::min();
constexpr decimal FIX_MAX = std::numeric_limits<decimal>::max();
constexpr decimal FIX_EPSILON = std::numeric_limits<decimal>::epsilon();
