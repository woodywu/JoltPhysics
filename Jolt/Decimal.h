// SPDX-FileCopyrightText: 2022 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <3rdParty/fpm/include/fpm/fixed.hpp>
#include <3rdParty/fpm/include/fpm/math.hpp>
#include <3rdParty/fpm/include/fpm/ios.hpp>

using decimal = fpm::fixed<std::int32_t, std::int64_t, 10>;

constexpr decimal C0 = decimal(0.0F);
constexpr decimal C0P5 = decimal(0.5F);
constexpr decimal C0P1 = decimal(0.1F);
constexpr decimal C1 = decimal(1.0F);
constexpr decimal C2 = decimal(2.0F);
constexpr decimal C3 = decimal(3.0F);

constexpr decimal FIX_MIN = std::numeric_limits<decimal>::min();
constexpr decimal FIX_MAX = std::numeric_limits<decimal>::max();
constexpr decimal FIX_EPSILON = std::numeric_limits<decimal>::epsilon();

namespace utils {
	using F = fpm::fixed<std::int32_t, std::int64_t, 10>;
};
