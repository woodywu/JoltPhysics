// SPDX-FileCopyrightText: 2022 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <3rdParty/fpm/include/fpm/fixed.hpp>
#include <3rdParty/fpm/include/fpm/math.hpp>
#include <3rdParty/fpm/include/fpm/ios.hpp>

using fbase_t = std::int32_t;
using fmedi_t = std::int64_t;
using decimal = fpm::fixed<std::int32_t, std::int64_t, 10>;
using decimal_raw = std::tuple<std::int64_t>;

constexpr decimal C0 = decimal(0.0F);
constexpr decimal C0P5 = decimal(0.5F);
constexpr decimal C0P1 = decimal(0.1F);
constexpr decimal C1 = decimal(1.0F);
constexpr decimal C2 = decimal(2.0F);
constexpr decimal C3 = decimal(3.0F);

static constexpr decimal FIX_MIN = std::numeric_limits<decimal>::min();
static constexpr decimal FIX_MAX = std::numeric_limits<decimal>::max();
static constexpr decimal FIX_EPSILON = std::numeric_limits<decimal>::epsilon();
static constexpr fmedi_t FIX_FRACTION_MULT = fmedi_t(1) << 10;

namespace utils {
	using F = fpm::fixed<std::int32_t, std::int64_t, 10>;

	static const decimal COS_179 = fpm::cos(decimal(179));
	static const decimal COS_5 = fpm::cos(decimal(5));
	static const decimal SIN_45 = fpm::sin(decimal(45));

	inline decimal R2D(const decimal_raw& raw) noexcept {
		fmedi_t val = std::get<0>(raw);
		assert(val <= std::numeric_limits<fbase_t>::max());
		assert(val >= std::numeric_limits<fbase_t>::min());
		return decimal::from_raw_value(static_cast<fbase_t>(val));
	}

	inline decimal_raw D2R(const decimal& val) noexcept {
		return val.raw_value();
	}

	constexpr inline decimal_raw radd(const decimal_raw& x, const decimal_raw& y) noexcept {
		return (std::get<0>(x) + std::get<0>(y));
	}

	constexpr inline decimal_raw rsub(const decimal_raw& x, const decimal_raw& y) noexcept {
		return (std::get<0>(x) - std::get<0>(y));
	}

	constexpr inline decimal_raw rmul(const decimal_raw& x, const decimal_raw& y) noexcept {
		return (std::get<0>(x) * std::get<0>(y)) / FIX_FRACTION_MULT;
	}

	constexpr inline decimal_raw rdiv(const decimal_raw& x, const decimal_raw& y) noexcept {
		return (std::get<0>(x) * FIX_FRACTION_MULT) / std::get<0>(y);
	}
};

