// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once
#include <format>

struct CollisionFreeSpeedModelData {
    double timeGap{1};
    double v0{1.2};
    double radius{0.2};
};

namespace std {
template <>
struct formatter<CollisionFreeSpeedModelData> {

    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const CollisionFreeSpeedModelData& m, FormatContext& ctx) const
    {
        return std::format_to(
            ctx.out(),
            "CollisionFreeSpeedModel[timeGap={}, v0={}, radius={}])",
            m.timeGap,
            m.v0,
            m.radius);
    }
};
} // namespace std
