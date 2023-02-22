#pragma once

#include <Libs/fpm/fixed.hpp>
#include <Libs/fpm/math.hpp>
#include <Libs/fpm/ios.hpp>

using decimal = fpm::fixed<std::int32_t, std::int64_t, 8>;

constexpr decimal C0 = decimal(0.0F);
constexpr decimal C0P5 = decimal(0.5F);
constexpr decimal C0P1 = decimal(0.1F);
constexpr decimal C1 = decimal(1.0F);
constexpr decimal C2 = decimal(2.0F);
constexpr decimal C3 = decimal(3.0F);

static constexpr decimal FIX_MIN = std::numeric_limits<decimal>::min();
static constexpr decimal FIX_MAX = std::numeric_limits<decimal>::max();
static constexpr decimal FIX_EPSILON = std::numeric_limits<decimal>::epsilon();

namespace utils {
	using F = decimal;

	static const decimal COS_179 = fpm::cos(decimal(179));
	static const decimal COS_5 = fpm::cos(decimal(5));
	static const decimal SIN_45 = fpm::sin(decimal(45));
};

