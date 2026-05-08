// SPDX-License-Identifier: LGPL-3.0-or-later
#include "Tracing.hpp"

#include "Logger.hpp"

#include <chrono>
#include <cstdint>
#include <fstream>
#include <optional>
#include <utility>

PERFETTO_TRACK_EVENT_STATIC_STORAGE();

namespace
{
perfetto::TraceConfig buildDefaultTraceConfig()
{
    perfetto::TraceConfig cfg;

    auto* buffer = cfg.add_buffers();
    buffer->set_size_kb(8192);

    auto* ds_cfg = cfg.add_data_sources()->mutable_config();
    ds_cfg->set_name("track_event");

    return cfg;
}
} // namespace

void Profiler::createSession()
{
    if(tracing_session) {
        return;
    }

    if(!perfetto::Tracing::IsInitialized()) {
        perfetto::TracingInitArgs args;
        args.backends |= perfetto::kInProcessBackend;
        perfetto::Tracing::Initialize(args);
    }

    perfetto::TrackEvent::Register();

    tracing_session = perfetto::Tracing::NewTrace();
    tracing_session->Setup(buildDefaultTraceConfig());
    tracing_session->StartBlocking();
}

void Profiler::writeAndResetSession(const std::string& filename)
{
    if(!tracing_session) {
        return;
    }

    perfetto::TrackEvent::Flush();
    tracing_session->StopBlocking();
    const auto trace_data = tracing_session->ReadTraceBlocking();
    if(filename.empty()) {
        tracing_session.reset();
        return;
    }
    std::ofstream output(filename, std::ios::binary | std::ios::trunc);
    if(output.is_open()) {
        output.write(trace_data.data(), static_cast<std::streamsize>(trace_data.size()));
        output.flush();
    } else {
        LOG_ERROR("Failed to open Perfetto output file: {}", filename);
    }

    tracing_session.reset();
}

void Profiler::enable()
{
    auto& p = instance();
    if(p.enabled) {
        return;
    }

    p.createSession();
    p.enabled = true;
}

void Profiler::disable()
{
    auto& p = instance();
    if(!p.enabled && !p.tracing_session) {
        return;
    }

    p.writeAndResetSession("");
    p.enabled = false;
}

void Profiler::dumpAndReset(const std::string& filename)
{
    auto& p = instance();
    p.writeAndResetSession(filename);
    p.enabled = false;
}

Profiler Profiler::profiler{};

namespace cr = std::chrono;

Trace::Trace(uint64_t& _t) : startedAt(cr::high_resolution_clock::now()), t(_t)
{
}

std::optional<Trace> PerfStats::trace(uint64_t& v)
{
    if(enabled) {
        return std::optional<Trace>{std::in_place, v};
    } else {
        return std::nullopt;
    }
}
std::optional<Trace> PerfStats::TraceIterate()
{
    return trace(iterate_duration);
}

std::optional<Trace> PerfStats::TraceOperationalDecisionSystemRun()
{
    return trace(op_dec_system_run_duration);
}
