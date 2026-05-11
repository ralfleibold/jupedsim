// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once
#include <format>

struct CollisionFreeSpeedModelV2Data {
    double strengthNeighborRepulsion{8.0};
    double rangeNeighborRepulsion{0.1};
    double strengthGeometryRepulsion{5.0};
    double rangeGeometryRepulsion{0.02};

    double timeGap{1};
    double v0{1.2};
    double radius{0.2};
};

namespace std {
template <>
struct formatter<CollisionFreeSpeedModelV2Data> {

    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const CollisionFreeSpeedModelV2Data& m, FormatContext& ctx) const
    {
        return std::format_to(
            ctx.out(),
            "CollisionFreeSpeedModelV2[strengthNeighborRepulsion={}, "
            "rangeNeighborRepulsion={}, strengthGeometryRepulsion={}, rangeGeometryRepulsion={}, "
            "timeGap={}, v0={}, radius={}])",
            m.strengthNeighborRepulsion,
            m.rangeNeighborRepulsion,
            m.strengthGeometryRepulsion,
            m.rangeGeometryRepulsion,
            m.timeGap,
            m.v0,
            m.radius);
    }
};
} // namespace std
