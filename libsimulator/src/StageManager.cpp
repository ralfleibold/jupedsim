// SPDX-License-Identifier: LGPL-3.0-or-later
#include "StageManager.hpp"

#include "Geometry/Geometry.hpp"
#include "Polygon.hpp"
#include "RoutingEngine.hpp"
#include "Visitor.hpp"

#include <cmath>
#include <numbers>
#include <string_view>

namespace
{
/// Put a stage's representative point on the surface that is closest to @p z_hint.
Location
locate_stage_point(const Geometry& geometry, Point point, std::string_view what, double z_hint)
{
    const auto location = geometry.get_location(point.x, point.y, z_hint);
    if(!location) {
        throw SimulationError("{} {} not inside walkable area", what, point);
    }
    return *location;
}

std::vector<Location> locate_slots(
    const Geometry& geometry,
    const std::vector<Point>& slots,
    std::string_view what,
    double z_hint)
{
    std::vector<Location> located{};
    located.reserve(slots.size());
    for(const auto& slot : slots) {
        located.push_back(locate_stage_point(geometry, slot, what, z_hint));
    }
    return located;
}

/// The circle of @p radius around @p center as a regular polygon.
Polygon circle2polygon(Point center, double radius)
{
    constexpr int corners = 16;
    std::vector<Point> points{};
    points.reserve(corners);
    for(int i = 0; i < corners; ++i) {
        const double angle = 2.0 * std::numbers::pi * i / corners;
        points.push_back(center + Point{std::cos(angle), std::sin(angle)} * radius);
    }
    return Polygon{points};
}
} // namespace

BaseStage::ID StageManager::AddStage(
    const StageDescription& stageDescription,
    std::vector<GenericAgent::ID>& removedAgentsInLastIteration,
    const Geometry& geometry,
    RoutingEngine& routingEngine,
    double z_hint)
{
    std::unique_ptr<BaseStage> stage = std::visit(
        overloaded{
            [&geometry, &routingEngine, z_hint](
                const WaypointDescription& d) -> std::unique_ptr<BaseStage> {
                if(d.distance <= 0.0) {
                    throw SimulationError("WayPoint distance must be positive, got {}", d.distance);
                }
                const auto position = locate_stage_point(geometry, d.position, "WayPoint", z_hint);
                const DestinationArea area{
                    position.region(), circle2polygon(d.position, d.distance)};
                return std::make_unique<Waypoint>(
                    position, d.distance, routingEngine.AddDestination({&area, 1}));
            },
            [&removedAgentsInLastIteration, &geometry, &routingEngine, z_hint](
                const ExitDescription& d) -> std::unique_ptr<BaseStage> {
                const auto centroid =
                    locate_stage_point(geometry, d.polygon.Centroid(), "Exit", z_hint);
                const DestinationArea area{centroid.region(), d.polygon};
                return std::make_unique<Exit>(
                    d.polygon,
                    centroid,
                    routingEngine.AddDestination({&area, 1}),
                    removedAgentsInLastIteration);
            },
            [&geometry,
             z_hint](const NotifiableWaitingSetDescription& d) -> std::unique_ptr<BaseStage> {
                return std::make_unique<NotifiableWaitingSet>(
                    locate_slots(geometry, d.slots, "NotifiableWaitingSet point", z_hint));
            },
            [&geometry, z_hint](const NotifiableQueueDescription& d) -> std::unique_ptr<BaseStage> {
                return std::make_unique<NotifiableQueue>(
                    locate_slots(geometry, d.slots, "NotifiableQueue point", z_hint));
            },
            [](const DirectSteeringDescription&) -> std::unique_ptr<BaseStage> {
                return std::make_unique<DirectSteering>();
            }},
        stageDescription);
    if(stages.find(stage->Id()) != stages.end()) {
        throw SimulationError("Internal error, stage id already in use.");
    }
    const auto id = stage->Id();
    stages.emplace(id, std::move(stage));

    return id;
}
