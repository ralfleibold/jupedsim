// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "Geometry/Location.hpp"
#include "Visitor.hpp"

#include <fmt/core.h>

#include <cstddef>
#include <variant>

enum class DestinationId : std::size_t {};

inline auto format_as(DestinationId id)
{
    return static_cast<std::size_t>(id);
}

using RoutingTarget = std::variant<DestinationId, Location>;

template <>
struct fmt::formatter<RoutingTarget> {
    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const RoutingTarget& target, FormatContext& ctx) const
    {
        return std::visit(
            overloaded{
                [&ctx](DestinationId id) {
                    return fmt::format_to(ctx.out(), "destination {}", id);
                },
                [&ctx](const Location& l) { return fmt::format_to(ctx.out(), "{}", l); }},
            target);
    }
};
