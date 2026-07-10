// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "GenericAgent.hpp"
#include "LineSegment.hpp"

#include <optional>
#include <span>
#include <vector>

/// What a model asks the framework to gather for one agent before calling into
/// the model (update step or constraint check). `std::nullopt` means "not
/// needed, do not compute". Queried per agent and per step -- requirements may
/// vary between agents and over time.
struct InformationRequirements {
    std::optional<double> neighborRadius{};
    std::optional<double> wallRadius{};
};

/// Per-agent input gathered by the framework according to the model's
/// InformationRequirements. Models are pure functions of (dT, agent, info) --
/// they never query the world themselves, so the same model runs on any world
/// (2D collision geometry, 3D surface mesh) that can fill this struct.
struct InformationForUpdate {
    /// All agents within neighborRadius of the agent's position, the agent
    /// itself included.
    std::vector<GenericAgent> neighbors{};
    /// All wall segments within wallRadius of the agent's position. May be
    /// over-inclusive (segments further away can appear); models must apply
    /// exact distances themselves. Points into framework-owned storage and is
    /// only valid for the duration of the model call.
    std::span<const LineSegment> walls{};
};
