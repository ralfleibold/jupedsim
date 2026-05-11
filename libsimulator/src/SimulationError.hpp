// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once
#include <format>

#include <stdexcept>

class SimulationError : public std::runtime_error
{
public:
    template <typename... Args>
    SimulationError(const char* msg, const Args&... args)
        : std::runtime_error(std::vformat(msg, std::make_format_args(args...)))
    {
    }
};
