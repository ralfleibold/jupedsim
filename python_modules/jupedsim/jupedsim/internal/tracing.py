# SPDX-License-Identifier: LGPL-3.0-or-later
from contextlib import contextmanager

import jupedsim.native as py_jps


class Trace:
    """
    .. important::

        This is indented for internal usage. We will not guarantee that this API will
        stable and available in any release. It might be changed on any update, regardless of
        a major/minor/patch update.
    """

    def __init__(self) -> None:
        self._obj = py_jps.Trace

    @property
    def iteration_duration(self) -> float:
        """Time for one simulation iteration in us.

        Returns:
             Time for one simulation iteration in us
        """
        return self._obj.iteration_duration

    @property
    def operational_level_duration(self) -> float:
        """Time for one simulation iteration in the operational level in us.

        Returns:
             Time for one simulation iteration in the operational level in us
        """

        return self._obj.operational_level_duration

    def __str__(self) -> str:
        return self._obj.__repr__()


def enable_tracing():
    py_jps.Profiler.enable()


def disable_tracing():
    py_jps.Profiler.disable()


def is_tracing_enabled():
    return py_jps.Profiler().is_enabled


def start_trace(name: str):
    py_jps.Profiler.start_trace_event(name)


def end_trace():
    py_jps.Profiler.end_trace_event()


def dump_traces(filename: str):
    py_jps.Profiler.dump_and_reset(filename)


@contextmanager
def trace_region(name: str):
    start_trace(name)
    try:
        yield
    finally:
        end_trace()
