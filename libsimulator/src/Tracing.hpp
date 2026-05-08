// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <perfetto.h>

PERFETTO_DEFINE_CATEGORIES(perfetto::Category("main").SetDescription("Main iteration over agents"));

#ifndef JPS_TRACE_EVENT
#define JPS_TRACE_EVENT TRACE_EVENT
#define JPS_TRACE_EVENT_BEGIN TRACE_EVENT_BEGIN
#define JPS_TRACE_EVENT_END TRACE_EVENT_END
#endif

#include <chrono>
#include <cstdint>
#include <optional>

class Profiler
{
    static Profiler profiler;

public:
    static Profiler& instance() noexcept { return profiler; };
    static void enable();
    static void disable();

    static void dumpAndReset(const std::string& filename);
    inline bool isEnabled() const { return enabled; }

private:
    Profiler() = default;
    Profiler(const Profiler&) = delete;
    Profiler& operator=(const Profiler&) = delete;
    Profiler(Profiler&&) = delete;
    Profiler& operator=(Profiler&&) = delete;

    void createSession();
    void writeAndResetSession(const std::string& filename);

    bool enabled{false};
    std::unique_ptr<perfetto::TracingSession> tracing_session{};
};

class Trace
{
    std::chrono::high_resolution_clock::time_point startedAt;
    uint64_t& t;

public:
    Trace(uint64_t& _t);
    ~Trace()
    {
        const auto now = std::chrono::high_resolution_clock::now();
        t = std::chrono::duration_cast<std::chrono::microseconds>(now - startedAt).count();
    }
    Trace(const Trace& other) = delete;
    Trace& operator=(const Trace& other) = delete;
    Trace(Trace&& other) = delete;
    Trace& operator=(const Trace&& other) = delete;
};

class PerfStats
{
    uint64_t iterate_duration{};
    uint64_t op_dec_system_run_duration{};
    bool enabled{false};

public:
    std::optional<Trace> TraceIterate();
    std::optional<Trace> TraceOperationalDecisionSystemRun();
    void SetEnabled(bool status) { enabled = status; };
    uint64_t IterationDuration() const { return iterate_duration; };
    uint64_t OpDecSystemRunDuration() const { return op_dec_system_run_duration; };

private:
    std::optional<Trace> trace(uint64_t& v);
};
